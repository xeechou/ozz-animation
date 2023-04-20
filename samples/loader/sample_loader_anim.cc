#include <algorithm>
#include <cstddef>
#include <string>
#include <unordered_map>
#include "ozz/animation/runtime/skeleton.h"
#include "sample_loader.h"
#include "ozz/animation/offline/animation_builder.h"


using RawSkeleton    = ozz::animation::offline::RawSkeleton;
using RawAnimation   = ozz::animation::offline::RawAnimation;
using TranslationKey = RawAnimation::TranslationKey;
using RotationKey    = RawAnimation::RotationKey;
using ScaleKey       = RawAnimation::ScaleKey;
using NodeMapping    = std::unordered_map<std::string, const tinygltf::Node*>;

static inline int32_t GetTypeSizeInBytes(uint32_t ty)
{
  if (ty == TINYGLTF_TYPE_SCALAR) {
    return 1;
  } else if (ty == TINYGLTF_TYPE_VEC2) {
    return 2;
  } else if (ty == TINYGLTF_TYPE_VEC3) {
    return 3;
  } else if (ty == TINYGLTF_TYPE_VEC4) {
    return 4;
  } else if (ty == TINYGLTF_TYPE_MAT2) {
    return 4;
  } else if (ty == TINYGLTF_TYPE_MAT3) {
    return 9;
  } else if (ty == TINYGLTF_TYPE_MAT4) {
    return 16;
  } else {
    // Unknown componenty type
    return -1;
  }
}

TranslationKey create_restpos_translation(const tinygltf::Node& _node)
{
  ozz::animation::offline::RawAnimation::TranslationKey key;
  key.time = 0.0f;

  if (_node.translation.empty())
    key.value = ozz::math::Float3::zero();
  else
    key.value = ozz::math::Float3(static_cast<float>(_node.translation[0]),
                                  static_cast<float>(_node.translation[1]),
                                  static_cast<float>(_node.translation[2]));
  return key;
}

RotationKey create_restpos_rotation(const tinygltf::Node& _node)
{
  ozz::animation::offline::RawAnimation::RotationKey key;
  key.time = 0.0f;

  if (_node.rotation.empty())
    key.value = ozz::math::Quaternion::identity();
  else
    key.value = ozz::math::Quaternion(static_cast<float>(_node.rotation[0]),
                                      static_cast<float>(_node.rotation[1]),
                                      static_cast<float>(_node.rotation[2]),
                                      static_cast<float>(_node.rotation[3]));
  return key;
}

ScaleKey create_restpos_scale(const tinygltf::Node& _node)
{
  ozz::animation::offline::RawAnimation::ScaleKey key;
  key.time = 0.0f;

  if (_node.scale.empty())
    key.value = ozz::math::Float3::one();
  else
    key.value = ozz::math::Float3(static_cast<float>(_node.scale[0]),
                                  static_cast<float>(_node.scale[1]),
                                  static_cast<float>(_node.scale[2]));
  return key;
}


// Returns the address of a gltf buffer given an accessor.
// Performs basic checks to ensure the data is in the correct format
template <typename T>
ozz::span<const T> BufferView(const tinygltf::Model& _model,
                              const tinygltf::Accessor& _accessor)
{
  const int32_t component_size =
      tinygltf::GetComponentSizeInBytes(_accessor.componentType);
  const int32_t element_size =
      component_size * GetTypeSizeInBytes(_accessor.type);
  if (element_size != sizeof(T)) {
    ozz::log::Err() << "Invalid buffer view access. Expected element size '"
                    << sizeof(T) << " got " << element_size << " instead."
                    << std::endl;
    return ozz::span<const T>();
  }

  const tinygltf::BufferView& bufferView =
      _model.bufferViews[_accessor.bufferView];
  const tinygltf::Buffer& buffer = _model.buffers[bufferView.buffer];
  const T* begin = reinterpret_cast<const T*>(
      buffer.data.data() + bufferView.byteOffset + _accessor.byteOffset);
  return ozz::span<const T>(begin, _accessor.count);
}

// Samples a linear animation channel
// There is an exact mapping between gltf and ozz keyframes so we just copy
// everything over.
template <typename _KeyframesType>
bool SampleLinearChannel(const tinygltf::Model& _model,
                         const tinygltf::Accessor& _output,
                         const ozz::span<const float>& _timestamps,
                         _KeyframesType* _keyframes)
{
  const size_t gltf_keys_count = _output.count;

  if (gltf_keys_count == 0) {
    _keyframes->clear();
    return true;
  }

  typedef typename _KeyframesType::value_type::Value ValueType;
  const ozz::span<const ValueType> values =
      BufferView<ValueType>(_model, _output);
  if (values.size_bytes() / sizeof(ValueType) != gltf_keys_count ||
      _timestamps.size() != gltf_keys_count) {
    ozz::log::Err() << "gltf format error, inconsistent number of keys."
                    << std::endl;
    return false;
  }

  _keyframes->reserve(_output.count);
  for (size_t i = 0; i < _output.count; ++i) {
    const typename _KeyframesType::value_type key{_timestamps[i], values[i]};
    _keyframes->push_back(key);
  }

  return true;
}

// Samples a step animation channel
// There are twice-1 as many ozz keyframes as gltf keyframes
template <typename _KeyframesType>
bool SampleStepChannel(const tinygltf::Model& _model,
                       const tinygltf::Accessor& _output,
                       const ozz::span<const float>& _timestamps,
                       _KeyframesType* _keyframes) {
  const size_t gltf_keys_count = _output.count;

  if (gltf_keys_count == 0) {
    _keyframes->clear();
    return true;
  }

  typedef typename _KeyframesType::value_type::Value ValueType;
  const ozz::span<const ValueType> values =
      BufferView<ValueType>(_model, _output);
  if (values.size_bytes() / sizeof(ValueType) != gltf_keys_count ||
      _timestamps.size() != gltf_keys_count) {
    ozz::log::Err() << "gltf format error, inconsistent number of keys."
                    << std::endl;
    return false;
  }

  // A step is created with 2 consecutive keys. Last step is a single key.
  size_t numKeyframes = gltf_keys_count * 2 - 1;
  _keyframes->resize(numKeyframes);

  for (size_t i = 0; i < _output.count; i++) {
    typename _KeyframesType::reference key = _keyframes->at(i * 2);
    key.time = _timestamps[i];
    key.value = values[i];

    if (i < _output.count - 1) {
      typename _KeyframesType::reference next_key = _keyframes->at(i * 2 + 1);
      next_key.time = nexttowardf(_timestamps[i + 1], 0.f);
      next_key.value = values[i];
    }
  }

  return true;
}

// Samples a hermite spline in the form
// p(t) = (2t^3 - 3t^2 + 1)p0 + (t^3 - 2t^2 + t)m0 + (-2t^3 + 3t^2)p1 + (t^3 -
// t^2)m1 where t is a value between 0 and 1 p0 is the starting point at t = 0
// m0 is the scaled starting tangent at t = 0
// p1 is the ending point at t = 1
// m1 is the scaled ending tangent at t = 1
// p(t) is the resulting point value
template <typename T>
T SampleHermiteSpline(float _alpha, const T& p0, const T& m0, const T& p1,
                      const T& m1)
{
  assert(_alpha >= 0.f && _alpha <= 1.f);

  const float t1 = _alpha;
  const float t2 = _alpha * _alpha;
  const float t3 = t2 * _alpha;

  // a = 2t^3 - 3t^2 + 1
  const float a = 2.0f * t3 - 3.0f * t2 + 1.0f;
  // b = t^3 - 2t^2 + t
  const float b = t3 - 2.0f * t2 + t1;
  // c = -2t^3 + 3t^2
  const float c = -2.0f * t3 + 3.0f * t2;
  // d = t^3 - t^2
  const float d = t3 - t2;

  // p(t) = a * p0 + b * m0 + c * p1 + d * m1
  T pt = p0 * a + m0 * b + p1 * c + m1 * d;
  return pt;
}

// Samples a cubic-spline channel, converts it into 

template <typename _KeyframesType>
bool SampleCubicSplineChannel(const tinygltf::Model& _model,
                              const tinygltf::Accessor& _output,
                              const ozz::span<const float>& _timestamps,
                              float _sampling_rate, float _duration,
                              _KeyframesType* _keyframes)
{
  (void)_duration;

  //for cublic-spline, no longer 1 to 1 mapping, one timestamp corresponds to 3
  //value: (in-tangent, value, out-tangent).
  assert(_output.count % 3 == 0);
  size_t gltf_keys_count = _output.count / 3;

  if (gltf_keys_count == 0) {
    _keyframes->clear();
    return true;
  }

  typedef typename _KeyframesType::value_type::Value ValueType;
  const ozz::span<const ValueType> values =
      BufferView<ValueType>(_model, _output);
  if (values.size_bytes() / (sizeof(ValueType) * 3) != gltf_keys_count ||
      _timestamps.size() != gltf_keys_count) {
    ozz::log::Err() << "gltf format error, inconsistent number of keys."
                    << std::endl;
    return false;
  }

  // Iterate keyframes at _sampling_rate steps, between first and last time
  // stamps.
  ozz::animation::offline::FixedRateSamplingTime fixed_it(
      _timestamps[gltf_keys_count - 1] - _timestamps[0], _sampling_rate);
  _keyframes->resize(fixed_it.num_keys());
  size_t cubic_key0 = 0;
  for (size_t k = 0; k < fixed_it.num_keys(); ++k) {
    const float time = fixed_it.time(k) + _timestamps[0];

    // Creates output key.
    typename _KeyframesType::value_type key;
    key.time = time;

    // Makes sure time is in between the correct cubic keyframes.
    while (_timestamps[cubic_key0 + 1] < time) {
      cubic_key0++;
    }
    assert(_timestamps[cubic_key0] <= time &&
           time <= _timestamps[cubic_key0 + 1]);

    // Interpolate cubic key
    const float t0 = _timestamps[cubic_key0];      // keyframe before time
    const float t1 = _timestamps[cubic_key0 + 1];  // keyframe after time
    const float alpha = (time - t0) / (t1 - t0);
    const ValueType& p0 = values[cubic_key0 * 3 + 1];
    const ValueType m0 = values[cubic_key0 * 3 + 2] * (t1 - t0); //out-tangent
    const ValueType& p1 = values[(cubic_key0 + 1) * 3 + 1];
    const ValueType m1 = values[(cubic_key0 + 1) * 3] * (t1 - t0);
    key.value = SampleHermiteSpline(alpha, p0, m0, p1, m1);      //in-tangent

    // Pushes interpolated key.
    _keyframes->at(k) = key;
  }

  return true;
}



template <typename _KeyframesType>
bool SampleChannel(const tinygltf::Model& _model,
                   const std::string& _interpolation,
                   const tinygltf::Accessor& _output,
                   const ozz::span<const float>& _timestamps,
                   float _sampling_rate, float _duration,
                   _KeyframesType* _keyframes) {
  bool valid = false;
  if (_interpolation == "LINEAR") {
    valid = SampleLinearChannel(_model, _output, _timestamps, _keyframes);
  } else if (_interpolation == "STEP") {
    valid = SampleStepChannel(_model, _output, _timestamps, _keyframes);
  } else if (_interpolation == "CUBICSPLINE") {
    valid = SampleCubicSplineChannel(_model, _output, _timestamps,
                                     _sampling_rate, _duration, _keyframes);
  } else {
    ozz::log::Err() << "Invalid or unknown interpolation type '"
                    << _interpolation << "'." << std::endl;
    valid = false;
  }

  // Check if sorted (increasing time, might not be stricly increasing).
  if (valid) {
    valid = std::is_sorted(_keyframes->begin(), _keyframes->end(),
                           [](typename _KeyframesType::const_reference _a,
                              typename _KeyframesType::const_reference _b) {
                             return _a.time < _b.time;
                           });
    if (!valid) {
      ozz::log::Log()
          << "gltf format error, keyframes are not sorted in increasing order."
          << std::endl;
    }
  }

  // Remove keyframes with strictly equal times, keeping the first one.
  if (valid) {
    auto new_end = std::unique(_keyframes->begin(), _keyframes->end(),
                               [](typename _KeyframesType::const_reference _a,
                                  typename _KeyframesType::const_reference _b) {
                                 return _a.time == _b.time;
                               });
    if (new_end != _keyframes->end()) {
      _keyframes->erase(new_end, _keyframes->end());

      ozz::log::Log() << "gltf format error, keyframe times are not unique. "
                         "Imported data were modified to remove keyframes at "
                         "consecutive equivalent times."
                      << std::endl;
    }
  }
  return valid;
}


bool sample_animation_channel(const tinygltf::Model& model,
                              const tinygltf::AnimationSampler& sampler,
                              const std::string& target_path,
                              float _sampling_rate, float* _duration,
                              RawAnimation::JointTrack* _track)
{
  // Validate interpolation type.
  if (sampler.interpolation.empty()) {
    ozz::log::Err() << "Invalid sampler interpolation." << std::endl;
    return false;
  }

  auto& input = model.accessors[sampler.input];
  assert(input.maxValues.size() == 1);

  // The max[0] property of the input accessor is the animation duration
  // this is required to be present by the spec:
  // "Animation Sampler's input accessor must have min and max properties
  // defined."
  const float duration = static_cast<float>(input.maxValues[0]);

  // If this channel's duration is larger than the animation's duration
  // then increase the animation duration to match.
  if (duration > *_duration) {
    *_duration = duration;
  }

  assert(input.type == TINYGLTF_TYPE_SCALAR);
  auto& output = model.accessors[sampler.output];
  assert(output.type == TINYGLTF_TYPE_VEC3 ||
         output.type == TINYGLTF_TYPE_VEC4);

  const ozz::span<const float> timestamps = BufferView<float>(model, input);
  if (timestamps.empty()) {
    return true;
  }

  // Builds keyframes.
  bool valid = false;
  if (target_path == "translation") {
    valid =
      SampleChannel(model, sampler.interpolation, output, timestamps,
                    _sampling_rate, duration, &_track->translations);
  } else if (target_path == "rotation") {
    valid =
      SampleChannel(model, sampler.interpolation, output, timestamps,
                    _sampling_rate, duration, &_track->rotations);
    if (valid) {
      // Normalize quaternions.
      for (auto& key : _track->rotations) {
        key.value = ozz::math::Normalize(key.value);
      }
    }
  } else if (target_path == "scale") {
    valid =
      SampleChannel(model, sampler.interpolation, output, timestamps,
                    _sampling_rate, duration, &_track->scales);
  } else {
    assert(false && "Invalid target path");
  }

  return valid;
}

static inline std::string valid_name(const std::string& name, size_t i)
{
  return name.empty() ? "unnamed-animation-" + std::to_string(i) : name;
}

static bool does_clip_touch_skin(const ozz::animation::Skeleton& skin,
                                 const tinygltf::Animation& input_animation,
                                 const tinygltf::Model& input)
{
  //this should work
  for (const auto& channel : input_animation.channels)
  {
    const auto& node = input.nodes[channel.target_node];
    if (std::find(skin.joint_names().begin(), skin.joint_names().end(),
                  node.name) != skin.joint_names().end())
      return true;
  }
  return false;
}

static bool load_raw_animation(RawAnimation& raw_anim,
                               const ozz::animation::Skeleton& skeleton,
                               const tinygltf::Animation& input_anim,
                               const tinygltf::Model& input,
                               const NodeMapping& node_mapping)
{
  float sampling_rate = 30.0f;
  static const std::set<std::string> valid_paths = {"translation", "rotation", "scale"};


  raw_anim.duration = 0.0f;
  raw_anim.tracks.resize(skeleton.num_joints());

  std::map<std::string, std::vector<const tinygltf::AnimationChannel*>> joint_channels;
    
  for (const auto& channel : input_anim.channels)
  {
    if (channel.target_node == -1)
      continue;
    if (valid_paths.find(channel.target_path) == valid_paths.end())
      continue;
    const auto& target_node = input.nodes[channel.target_node];
    joint_channels[target_node.name].push_back(&channel);
  }

  //you cannot query based on skeleton's names though.
  const auto joint_names = skeleton.joint_names();
  //instead load based on joints, 

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    auto channels = joint_channels.find(joint_names[i]);
    auto& track = raw_anim.tracks[i];

    if (channels == joint_channels.end())
      continue;

    for (auto& channel : channels->second)
    {
      const auto& sampler = input_anim.samplers[channel->sampler];
      if (!sample_animation_channel(input, sampler, channel->target_path,
                                    sampling_rate, &raw_anim.duration, &track))
        return false;
    }

    const tinygltf::Node* node = node_mapping.at(joint_names[i]);

    // Pads the rest pose transform for any joints which do not have an
    // associated channel for this animation
    if (track.translations.empty())
      track.translations.push_back(create_restpos_translation(*node));

    if (track.rotations.empty())
      track.rotations.push_back(create_restpos_rotation(*node));

    if (track.scales.empty())
      track.scales.push_back(create_restpos_scale(*node));
  }

  if (!raw_anim.Validate())
  {
    throw std::runtime_error("failed to build the animation: " +
                             std::string(raw_anim.name));
    return false;
  }
  return true;
}

bool GLTFImporter::load_animations(const tinygltf::Model& input)
{
  //keeping a cache of node name mapping 
  NodeMapping node_mapping;
  for (size_t i = 0; i < input.nodes.size(); i++)
  {
    const auto& node = input.nodes[i];
    node_mapping[node.name] = &node;
  }

  //for every clip, we check which skin it touches, then we append this clip
  //into the joint's animation list to read.
  std::map<int, std::vector<int>> animation_list;
  for (size_t i = 0; i < input.animations.size(); i++)
  {
    const auto& anim = input.animations[i];
    for (size_t j = 0; j < m_skins.size(); j++)
    {
      if (does_clip_touch_skin(m_skins[j], anim, input))
        animation_list[j].push_back(i);
    }
  }

  for (auto itr : animation_list)
  {
    ozz::animation::Skeleton& skeleton = m_skins[itr.first];
    for (size_t idx = 0; idx < itr.second.size(); idx++)
    {
      const auto&  input_anim = input.animations[idx];
      RawAnimation raw_anim;

      raw_anim.name = valid_name(input_anim.name, m_clips.size());
      if (!load_raw_animation(raw_anim, skeleton, input_anim, input, node_mapping))
        return false;
      
      // Build the run time animation from the raw animation.
      ozz::animation::offline::AnimationBuilder animation_builder;
      auto animation = animation_builder(raw_anim);
      if (!animation) {
        return false;
      }
      m_clips.emplace_back(std::move(*animation));
    }
  }
  

  return true;

}
