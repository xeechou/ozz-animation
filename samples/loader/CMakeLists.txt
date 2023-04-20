add_executable(sample_loader
  sample_loader.cc
  sample_loader_skin.cc
  sample_loader_anim.cc
  main.cpp
)

target_link_libraries(sample_loader
  sample_framework)

target_include_directories(sample_loader PRIVATE glm tinygltf)

target_compile_features(sample_loader PRIVATE cxx_std_17)

target_copy_shared_libraries(sample_loader)
