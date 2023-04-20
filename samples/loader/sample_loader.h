#pragma once

#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/raw_animation_utils.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/offline/tools/import2ozz.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/containers/map.h"
#include "ozz/base/containers/set.h"
#include "ozz/base/containers/vector.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/math_ex.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/transform.h"
#include "ozz/base/memory/unique_ptr.h"

#include <utility>
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
//TODO remove this and use filesystem
#include <filesystem>
#include <vector>

// No support for image loading or writing
// #define TINYGLTF_NO_STB_IMAGE
// #define TINYGLTF_NO_STB_IMAGE_WRITE
// #define TINYGLTF_NO_EXTERNAL_IMAGE

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4702)  // unreachable code
#pragma warning(disable : 4267)  // conversion from 'size_t' to 'type'
#endif                           // _MSC_VER

#include "tiny_gltf.h"

#ifdef _MSC_VER
#pragma warning(pop)
#endif  // _MSC_VER


class GLTFImporter
{
public:
  bool operator()(const char* filename);

  const std::vector<ozz::animation::Skeleton>& skins()  const { return m_skins; }
  const std::vector<ozz::animation::Animation>& clips() const { return m_clips; }

protected:

  bool load_file(const std::string& filename, tinygltf::Model& model,
                 tinygltf::TinyGLTF& context);


  //! loading all the textures store in m_textures, handle the %20 as space
  bool load_meshes(const tinygltf::Model& model);
  bool load_textures(const tinygltf::Model& model);
  bool load_materials(const tinygltf::Model& model);
  bool load_nodes(const tinygltf::Model& model, const std::vector<int>& skin_roots);
  bool load_skins(const tinygltf::Model& model, std::vector<int>& skin_roots);
  bool load_cameras(const tinygltf::Model& model);
  bool load_lights(const tinygltf::Model& model);
  bool load_animations(const tinygltf::Model& model);


  ozz::animation::Skeleton& scene_tree() { return m_skins.back(); }

private:
  //the last one is the scene tree, problem is that ozz has limitation of 1000 bones.

  //maybe we can store the animation of the scene using tracks?
  std::vector<ozz::animation::Skeleton>  m_skins;
  std::vector<ozz::animation::Animation> m_clips;
};
