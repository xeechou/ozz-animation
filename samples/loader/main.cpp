#include <algorithm>
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/containers/vector.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/options/options.h"

#include "sample_loader.h"

#include "framework/application.h"
#include "framework/imgui.h"
#include "framework/renderer.h"
#include "framework/utils.h"


OZZ_OPTIONS_DECLARE_STRING(path,
                           "path to the gltf file to load", "sample.glb", false)

class SampleLoaderApplication : public ozz::sample::Application
{
public:
  SampleLoaderApplication(const char *path) : m_gltf_path(path) {}

protected:
  virtual bool OnUpdate(float dt, float) override {
    m_controller.Update(*m_animation, dt);

    ozz::animation::SamplingJob sampling_job;
    sampling_job.animation = m_animation;
    sampling_job.context = &m_context;
    sampling_job.ratio = m_controller.time_ratio();
    sampling_job.output = ozz::make_span(m_locals);
    if (!sampling_job.Run()) {
      return false;
    }

    ozz::animation::LocalToModelJob ltm_job;
    ltm_job.skeleton = m_skeleton;
    ltm_job.input = make_span(m_locals);
    ltm_job.output = make_span(m_models);
    if (!ltm_job.Run()) {
      return false;
    }

    return true;
  }

  virtual bool OnDisplay(ozz::sample::Renderer* _renderer) override {
    return _renderer->DrawPosture(*m_skeleton, make_span(m_models),
                                  ozz::math::Float4x4::identity());
  }


  virtual void OnDestroy() override {}

  virtual void GetSceneBounds(ozz::math::Box* _bound) const override {
    ozz::sample::ComputePostureBounds(make_span(m_models), _bound);
  }

  virtual bool OnGui(ozz::sample::ImGui* _im_gui) override
  {
    static bool open = true;
    ozz::sample::ImGui::OpenClose oc(_im_gui, "Playback control", &open);
    if (open)
    {
      _im_gui->DoLabel("Main layer:");
      _im_gui->DoSlider("selected animation", 0, (int)m_importer.clips().size(),
                        &m_selected_clip);
      m_animation = &m_importer.clips()[m_selected_clip];
      m_skeleton  = &m_importer.skins()[m_selected_clip];

      m_controller.OnGui(*m_animation, _im_gui);
    }
    return true;
  }

  virtual bool OnInitialize() override {

    if (!m_importer(OPTIONS_path))
      return false;

    int num_soa_joints = 0, num_joints = 0;
    std::for_each(m_importer.skins().begin(), m_importer.skins().end(),
                  [&num_soa_joints](const ozz::animation::Skeleton& skin) {
                    num_soa_joints = std::max(num_soa_joints,
                                              skin.num_soa_joints()); });
    std::for_each(m_importer.skins().begin(), m_importer.skins().end(),
                  [&num_joints](const ozz::animation::Skeleton& skin) {
                    num_joints = std::max(num_joints,
                                          skin.num_joints()); });
    m_models.resize(num_joints);
    m_context.Resize(num_joints);
    m_locals.resize(num_soa_joints);

    m_animation = &m_importer.clips()[m_selected_clip];
    m_skeleton  = &m_importer.skins()[m_selected_clip];


    return true;
  }

private:
  GLTFImporter m_importer;

  int m_selected_clip = 0;
  const ozz::animation::Animation *m_animation;
  const ozz::animation::Skeleton  *m_skeleton;

  ozz::sample::PlaybackController m_controller;

  //so we need to get the maximum required size
  ozz::animation::SamplingJob::Context m_context;
  ozz::vector<ozz::math::SoaTransform> m_locals;
  ozz::vector<ozz::math::Float4x4> m_models;

  const std::string m_gltf_path;
};


int main(int argc, const char *argv[])
{
  const char *title = "sample loader";
  return SampleLoaderApplication(argv[0]).Run(argc, argv, "1.0", title);
}
