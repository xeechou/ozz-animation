#include "sample_loader.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>


using RawSkeleton = ozz::animation::offline::RawSkeleton;
using RawAnimation = ozz::animation::offline::RawAnimation;


static void log_hierarchy(const RawSkeleton::Joint::Children& _children, int _depth = 0)
{
  const std::streamsize pres = ozz::log::LogV().stream().precision();
  for (size_t i = 0; i < _children.size(); ++i) {
    const auto& joint = _children[i];
    ozz::log::LogV() << std::setw(_depth) << std::setfill('.') << "";
    ozz::log::LogV() << joint.name.c_str() << std::setprecision(4)
                     << " t: " << joint.transform.translation.x << ", "
                     << joint.transform.translation.y << ", "
                     << joint.transform.translation.z
                     << " r: " << joint.transform.rotation.x << ", "
                     << joint.transform.rotation.y << ", "
                     << joint.transform.rotation.z << ", "
                     << joint.transform.rotation.w
                     << " s: " << joint.transform.scale.x << ", "
                     << joint.transform.scale.y << ", "
                     << joint.transform.scale.z << std::endl;

    // Recurse
    log_hierarchy(joint.children, _depth + 1);
  }
  ozz::log::LogV() << std::setprecision(static_cast<int>(pres));
}

typedef ozz::set<const char*, ozz::str_less> names;

static bool validate_unique_joint_names_recurse(const RawSkeleton::Joint::Children& _joints,
                                                names* _names)
{
  for (size_t i = 0; i < _joints.size(); ++i) {
    const RawSkeleton::Joint& joint = _joints[i];
    const char* name = joint.name.c_str();
    if (!_names->insert(name).second) {
      ozz::log::Err()
          << "Skeleton contains at least one non-unique joint name \"" << name
          << "\", which is not supported." << std::endl;
      return false;
    }
    if (!validate_unique_joint_names_recurse(_joints[i].children, _names)) {
      return false;
    }
  }
  return true;
}

static bool validate_unique_joint_names(const RawSkeleton& _skeleton)
{
  names joint_names;
  return validate_unique_joint_names_recurse(_skeleton.roots, &joint_names);
}


static bool load_joint_transform(ozz::math::Transform& transform,
                                 const tinygltf::Node& input_node)
{
  glm::quat rotation;
  glm::vec3 scaling, translation;

  transform = ozz::math::Transform::identity();
  if (!input_node.matrix.empty())
  {
    glm::vec4 perspective; glm::vec3 skew;
    glm::mat4 model_matrix =
      glm::make_mat4x4(input_node.matrix.data());
    glm::decompose(model_matrix, scaling, rotation, translation,
                   skew, perspective);
  }
  else //! getting each transformation
  {
    if (input_node.translation.size() == 3)
    {
      translation = glm::make_vec3(input_node.translation.data());
    }
    if (input_node.rotation.size() == 4)
    {
      rotation = glm::make_quat(input_node.rotation.data());
    }
    if (input_node.scale.size() == 3)
    {
      scaling = glm::make_vec3(input_node.scale.data());
    }
  }

  transform.translation =
    ozz::math::Float3(static_cast<float>(translation[0]),
                      static_cast<float>(translation[1]),
                      static_cast<float>(translation[2]));
  transform.rotation =
    ozz::math::Quaternion(static_cast<float>(rotation.x),
                          static_cast<float>(rotation.y),
                          static_cast<float>(rotation.z),
                          static_cast<float>(rotation.w));
  transform.scale = ozz::math::Float3(static_cast<float>(scaling[0]),
                                      static_cast<float>(scaling[1]),
                                      static_cast<float>(scaling[2]));
  return true;
}

/// find the root node in the
static bool load_skin_node(ozz::animation::offline::RawSkeleton::Joint& joint,
                           const tinygltf::Node& input_node,
                           const tinygltf::Model& input,
                           const std::vector<int>& skip_nodes)
{
    joint.name = input_node.name.c_str();

    if (!load_joint_transform(joint.transform, input_node))
    {
        return false;
    }
    joint.children.resize(input_node.children.size());

    for (size_t i = 0; i < input_node.children.size(); i++)
    {
        const auto& child_node = input.nodes[input_node.children[i]];
        auto& child_joint = joint.children[i];
        //we skip those nodes because they are part of the skeleton.
        if (std::find(skip_nodes.begin(), skip_nodes.end(),
                      input_node.children[i]) != skip_nodes.end())
        {
          continue;
        }
        if (!load_skin_node(child_joint, child_node, input, skip_nodes))
        {
            return false;
        }
    }
    return true;
}

static int find_skin_root(const tinygltf::Skin& skin, const tinygltf::Model& input)
{
  static constexpr int no_parent = -1;

  std::map<int, int> parents;
  std::set<int> joints;
  for (auto joint : skin.joints)
  {
    parents[joint] = no_parent;
    joints.insert(joint);
  }

  for (auto joint : skin.joints)
  {
    for (int child : input.nodes[joint].children)
    {
      parents[child] = joint;
    }
  }
  int root = skin.joints[0];
  while (parents[root] != no_parent) {
    root = parents[root];
  }
  return root;
}


bool GLTFImporter::load_skins(const tinygltf::Model& input,
                              std::vector<int>& skin_roots)
{
  //we can load the raw skeleton here, then
  //then use the SkeletonBuilder to build the true skeleton.

  //we import all the skeleton and a root skeleton.

  skin_roots.clear();
  for (const auto& skin : input.skins)
  {
    int skin_root = skin.skeleton;
    if (skin_root < 0)
      skin_root = find_skin_root(skin, input);
    skin_roots.push_back(skin_root);

    ozz::animation::offline::RawSkeleton raw_skeleton;
    raw_skeleton.roots.resize(1);


    load_skin_node(raw_skeleton.roots[0], input.nodes[skin_root], input, {});

    if (!raw_skeleton.Validate())
    {
      throw std::runtime_error("Unable to import the skin " +
                               std::to_string(skin_root));
      return false;
    }
    if (!validate_unique_joint_names(raw_skeleton))
    {
      throw std::runtime_error("joint name has to be unique for skin: " +
                               std::to_string(skin_root));
      return false;
    }

    if (ozz::log::GetLevel() == ozz::log::kVerbose)
    {
      log_hierarchy(raw_skeleton.roots);
    }
    ozz::unique_ptr<ozz::animation::Skeleton> skeleton;
    ozz::animation::offline::SkeletonBuilder builder;
    skeleton = builder(raw_skeleton);
    if (!skeleton)
    {
      throw std::runtime_error("unable to build skin");
      return false;
    }
    m_skins.emplace_back(std::move(*skeleton));
  }

  return true;
}

bool GLTFImporter::load_nodes(const tinygltf::Model& input,
                              const std::vector<int>& skin_roots)
{
  //I should actually do it here though.
  std::vector<int> scene_roots = input.scenes[input.defaultScene].nodes;

  //loading the scene as a skeleton.
  ozz::animation::offline::RawSkeleton raw_skeleton; //scene skeleton
  raw_skeleton.roots.resize(scene_roots.size());
  for (size_t i = 0; i < scene_roots.size(); i++)
  {
    const auto& root_node = input.nodes[scene_roots[i]];
    auto& root_joint = raw_skeleton.roots[i];
    if (!load_skin_node(root_joint, root_node, input, skin_roots))
    {
      return false;
    }
  }
  if (!raw_skeleton.Validate())
  {
    throw std::runtime_error("Unable to import the scene");
    return false;
  }
  if (!validate_unique_joint_names(raw_skeleton))
  {
    throw std::runtime_error("joint name has to be unique for scene nodes");
    return false;
  }
  ozz::unique_ptr<ozz::animation::Skeleton> skeleton;
  ozz::animation::offline::SkeletonBuilder builder;
  skeleton = builder(raw_skeleton);
  if (!skeleton)
  {
    throw std::runtime_error("unable to build skin");
    return false;
  }
  m_skins.emplace_back(std::move(*skeleton)); //last one is the scene node

  return true;
}
