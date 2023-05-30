//----------------------------------------------------------------------------//
//                                                                            //
// ozz-animation is hosted at http://github.com/guillaumeblanc/ozz-animation  //
// and distributed under the MIT License (MIT).                               //
//                                                                            //
// Copyright (c) Guillaume Blanc                                              //
//                                                                            //
// Permission is hereby granted, free of charge, to any person obtaining a    //
// copy of this software and associated documentation files (the "Software"), //
// to deal in the Software without restriction, including without limitation  //
// the rights to use, copy, modify, merge, publish, distribute, sublicense,   //
// and/or sell copies of the Software, and to permit persons to whom the      //
// Software is furnished to do so, subject to the following conditions:       //
//                                                                            //
// The above copyright notice and this permission notice shall be included in //
// all copies or substantial portions of the Software.                        //
//                                                                            //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR //
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   //
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    //
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER //
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    //
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        //
// DEALINGS IN THE SOFTWARE.                                                  //
//                                                                            //
//----------------------------------------------------------------------------//

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "framework/application.h"
#include "framework/imgui.h"
#include "framework/renderer.h"
#include "framework/utils.h"
#include "ozz/animation/offline/animation_builder.h"
#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/maths/internal/simd_math_config.h"
#include "ozz/base/maths/math_constant.h"
#include "ozz/base/maths/quaternion.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/base/span.h"

using ozz::animation::offline::RawAnimation;
using ozz::animation::offline::RawSkeleton;
using ozz::math::Float3;
using ozz::math::Float4;
using ozz::math::Float4x4;
using ozz::math::Quaternion;
using ozz::math::SoaTransform;

// A millipede slice is 2 legs and a spine.
// Each slice is made of 7 joints, organized as follows.
//          * root
//             |
//           spine                                   spine
//         |       |                                   |
//     left_up    right_up        left_down - left_u - . - right_u - right_down
//       |           |                  |                                    |
//   left_down     right_down     left_foot         * root            right_foot
//     |               |
// left_foot        right_foot

// The following constants are used to define the millipede skeleton and
// animation.
// Skeleton constants.
const Float3 kTransUp = Float3(0.f, 1.f, 0.f);
const Float3 kTransDown = Float3(0.f, 0.f, 1.f);
const Float3 kTransFoot = Float3(1.f, 0.f, 0.f);

const Quaternion kRotLeftUp =
    Quaternion::FromAxisAngle(Float3::y_axis(), -ozz::math::kPi_2);
const Quaternion kRotLeftDown =
    Quaternion::FromAxisAngle(Float3::x_axis(), ozz::math::kPi_2) *
    Quaternion::FromAxisAngle(Float3::y_axis(), -ozz::math::kPi_2);
const Quaternion kRotRightUp =
    Quaternion::FromAxisAngle(Float3::y_axis(), ozz::math::kPi_2);
const Quaternion kRotRightDown =
    Quaternion::FromAxisAngle(Float3::x_axis(), ozz::math::kPi_2) *
    Quaternion::FromAxisAngle(Float3::y_axis(), -ozz::math::kPi_2);

class MillipedeSampleApplication : public ozz::sample::Application {
 public:
  MillipedeSampleApplication() : joint_count_(3), curr_joint_(0) {}

 protected:
  virtual bool OnUpdate(float _dt, float) {
    const ozz::math::SimdFloat4 w_axis = ozz::math::simd_float4::w_axis();
    ozz::math::SimdFloat4 trans =
        ozz::math::simd_float4::Load3PtrU(&kTransUp.x);

    ozz::vector<ozz::math::Float4x4> transforms(joint_count_);
    for (int i = 0; i < joint_count_; i++) {
      ozz::math::SimdFloat4 rotate = ozz::math::NormalizeSafe4(
          ozz::math::simd_float4::LoadPtrU(&rotations_[i].x), w_axis);

      transforms[i] = ozz::math::Float4x4::Translation(trans) *
                      ozz::math::Float4x4::FromEuler(rotate);
    }
    // now we need to convert it into locals_
    for (size_t i = 0; i < models_.size(); i++) {
      models_[i] = (i == 0) ? transforms[i] : transforms[i] * models_[i - 1];
    }

    // // Converts from local space to model space matrices.
    // ozz::animation::LocalToModelJob ltm_job;
    // ltm_job.skeleton = skeleton_.get();
    // ltm_job.input = make_span(locals_);
    // ltm_job.output = make_span(models_);
    return true;
  }

  virtual bool OnDisplay(ozz::sample::Renderer* _renderer) {
    // Renders the animated posture.
    bool success = _renderer->DrawPosture(*skeleton_, ozz::make_span(models_),
                                          ozz::math::Float4x4::identity());
    success &= _renderer->DrawAxes(models_[curr_joint_] * Float4x4::identity());
    return success;
  }

  virtual bool OnInitialize() { return Build(); }

  virtual void OnDestroy() {}

  virtual bool OnGui(ozz::sample::ImGui* _im_gui) {
    // Rebuilds all if the number of joints has changed.
    int joints = skeleton_->num_joints();
    char label[64];
    std::sprintf(label, "Joints count: %d", joints);

    _im_gui->DoSlider("Current Joint", 0, joint_count_ - 1, &curr_joint_);

    Float3 eulers;
    Quaternion& curr_rotation = rotations_[curr_joint_];
    eulers = ozz::math::ToEuler(curr_rotation);
    _im_gui->DoSlider("yaw", 0.0f, ozz::math::kPi, &eulers.x);
    _im_gui->DoSlider("pitch", 0.0f, ozz::math::kPi, &eulers.y);
    _im_gui->DoSlider("roll", 0.0, ozz::math::kPi, &eulers.z);
    curr_rotation = Quaternion::FromEuler(eulers.x, eulers.y, eulers.z);

    // Uses an exponential scale in the slider to maintain enough precision in
    // the lowest values.
    if (_im_gui->DoSlider(label, 8, ozz::animation::Skeleton::kMaxJoints,
                          &joints, .3f, true)) {
      const int new_joints_count = (joints - 1);
      // Slider use floats, we need to check if it has really changed.
      if (new_joints_count != joint_count_) {
        joint_count_ = new_joints_count;
        if (!Build()) {
          return false;
        }
      }
    }

    return true;
  }

  // Procedurally builds millipede skeleton and walk animation
  bool Build() {
    // Initializes the root. The root pointer will change from a spine to the
    // next for each slice.
    RawSkeleton raw_skeleton;
    CreateSkeleton(&raw_skeleton);
    const int num_joints = raw_skeleton.num_joints();
    rotations_.resize(num_joints);
    std::fill(rotations_.begin(), rotations_.end(), Quaternion::identity());

    // Build the run time skeleton.
    ozz::animation::offline::SkeletonBuilder skeleton_builder;
    skeleton_ = skeleton_builder(raw_skeleton);
    if (!skeleton_) {
      return false;
    }

    // Allocates runtime buffers.
    const int num_soa_joints = skeleton_->num_soa_joints();
    locals_.resize(num_soa_joints);
    models_.resize(num_joints);

    // Allocates a context that matches new animation requirements.
    context_.Resize(num_joints);

    return true;
  }

  void CreateSkeleton(ozz::animation::offline::RawSkeleton* _skeleton) {
    _skeleton->roots.resize(1);
    RawSkeleton::Joint* root = &_skeleton->roots[0];
    root->name = "root";
    root->transform.translation = Float3(0.f, 0.1f, 0.0);
    root->transform.rotation = Quaternion::identity();
    root->transform.scale = Float3::one();

    char buf[16];
    for (int i = 0; i < joint_count_; ++i) {
      root->children.resize(1);
      // Format joint number.
      std::sprintf(buf, "joint-%d", i);

      RawSkeleton::Joint& joint = root->children[0];
      joint.name = buf;
      joint.transform.translation = Float3(0.0f, 1.0f, 0.0f);
      joint.transform.rotation = Quaternion::identity();
      joint.transform.scale = Float3::one();
      root = &root->children[0];
    }
  }

  void CreateAnimation(ozz::animation::offline::RawAnimation* _animation) {}

  virtual void GetSceneBounds(ozz::math::Box* _bound) const {
    ozz::sample::ComputePostureBounds(make_span(models_), _bound);
  }

 private:
  // Playback animation controller. This is a utility class that helps with
  // controlling animation playback time.
  ozz::sample::PlaybackController controller_;

  // Millipede skeleton number of slices. 7 joints per slice.
  int joint_count_;
  int curr_joint_;

  // The millipede skeleton.
  ozz::unique_ptr<ozz::animation::Skeleton> skeleton_;

  // Sampling context, as used by SamplingJob.
  ozz::animation::SamplingJob::Context context_;

  ozz::vector<ozz::math::Quaternion> rotations_;
  // Buffer of local transforms as sampled from animation_.
  // These are shared between sampling output and local-to-model input.
  ozz::vector<ozz::math::SoaTransform> locals_;

  // Buffer of model matrices (local-to-model output).
  ozz::vector<ozz::math::Float4x4> models_;
};

int main(int _argc, const char** _argv) {
  const char* title = "Ozz-animation sample: RawAnimation/RawSkeleton building";
  return MillipedeSampleApplication().Run(_argc, _argv, "1.0", title);
}
