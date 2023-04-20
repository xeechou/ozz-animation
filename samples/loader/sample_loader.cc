#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "sample_loader.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>

// The start of importing process starting from OzzImporter::operator(argc,
// argv). Internally, it will call the Load(file name) function, then inside,
// it's calling two function. Import a skeleton, and then import animations. So
// it basically assume we only have one skeleton.

namespace fs = std::filesystem;

using RawSkeleton = ozz::animation::offline::RawSkeleton;
using RawAnimation = ozz::animation::offline::RawAnimation;

//okay, I don't need the OzzImporter here, it's basically a operator, I need to
//gather the information only

bool GLTFImporter::operator()(const char *filename)
{
    bool success = false;
    tinygltf::Model gltf_model;
    tinygltf::TinyGLTF gltf_context;
    std::vector<int> skin_roots;

    success = load_file(filename, gltf_model, gltf_context);
    success = success && load_textures(gltf_model);
    success = success && load_materials(gltf_model);
    //load skin before mesh so we can remap node to skin node.
    success = success && load_skins(gltf_model, skin_roots);
    success = success && load_meshes(gltf_model);

    success = success && load_cameras(gltf_model);
    success = success && load_lights(gltf_model);

    success = success && load_nodes(gltf_model, skin_roots);
    //animation comes at last since it drives the node
    success = success && load_animations(gltf_model);

    return success;
}

bool GLTFImporter::load_file(const std::string& filename, tinygltf::Model& model,
                             tinygltf::TinyGLTF& context)
{
  bool success = false;

  std::string errors;
  std::string warnings;


  const fs::path ext = fs::path(filename).extension();

  if (!fs::exists(filename))
  {
    throw std::invalid_argument("file: " + filename + " does not exist.");
  }
  if (ext.string() == ".glb")
  {
    success = context.LoadBinaryFromFile(&model, &errors, &warnings, filename);
  }
  else
  {
    if (ext.string() != ".gltf")
    {
      ozz::log::Log() << "Uknown file extension" << ext.string()
                      << ", assuming a JSON-Format gltf." << std::endl;
    }
    success = context.LoadASCIIFromFile(&model, &errors, &warnings, filename);
  }

  // Prints any errors or warnings emitted by the loader
  if (!warnings.empty())
  {
    ozz::log::Log() << "glTF parsing warnings: " << warnings << std::endl;
  }

  if (!errors.empty())
  {
    ozz::log::Err() << "glTF parsing errors: " << errors << std::endl;
  }

  if (success)
  {
    ozz::log::Log() << "glTF parsed successfully." << std::endl;
  }

  // if (success)
  // {
  //   success &= FixupNames(m_model.scenes, "Scene", "scene_");
  //   success &= FixupNames(m_model.nodes, "Node", "node_");
  //   success &= FixupNames(m_model.animations, "Animation", "animation_");
  // }

  return success;
}

bool GLTFImporter::load_meshes(const tinygltf::Model& model){return true;}
bool GLTFImporter::load_textures(const tinygltf::Model& model){return true;}
bool GLTFImporter::load_materials(const tinygltf::Model& model){return true;}
bool GLTFImporter::load_cameras(const tinygltf::Model& model){return true;}
bool GLTFImporter::load_lights(const tinygltf::Model& model){return true;}


// int main(int argc, const char *argv[])
// {
//   // const char *title = "sample loader";
//   GLTFImporter importer;
//   return importer(argv[1]);
// }
