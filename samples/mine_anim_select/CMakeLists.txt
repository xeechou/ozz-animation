# Generates sample data

add_custom_command(
  DEPENDS $<$<BOOL:${ozz_build_fbx}>:BUILD_DATA>
          "${ozz_media_directory}/bin/pab_skeleton.ozz"
          "${ozz_media_directory}/bin/pab_walk.ozz"
          "${ozz_media_directory}/bin/pab_jog.ozz"
          "${ozz_media_directory}/bin/pab_run.ozz"
  OUTPUT  "${CMAKE_CURRENT_BINARY_DIR}/media/skeleton.ozz"
          "${CMAKE_CURRENT_BINARY_DIR}/media/animation1.ozz"
          "${CMAKE_CURRENT_BINARY_DIR}/media/animation2.ozz"
          "${CMAKE_CURRENT_BINARY_DIR}/media/animation3.ozz"
  COMMAND ${CMAKE_COMMAND} -E make_directory media
  COMMAND ${CMAKE_COMMAND} -E copy "${ozz_media_directory}/bin/pab_skeleton.ozz" "./media/skeleton.ozz"
  COMMAND ${CMAKE_COMMAND} -E copy "${ozz_media_directory}/bin/pab_walk.ozz" "./media/animation1.ozz"
  COMMAND ${CMAKE_COMMAND} -E copy "${ozz_media_directory}/bin/pab_jog.ozz" "./media/animation2.ozz"
  COMMAND ${CMAKE_COMMAND} -E copy "${ozz_media_directory}/bin/pab_run.ozz" "./media/animation3.ozz"
  VERBATIM)

# Adds sample executable
add_executable(sample_select
  sample_select.cc
  "${CMAKE_CURRENT_BINARY_DIR}/media/skeleton.ozz"
  "${CMAKE_CURRENT_BINARY_DIR}/media/animation1.ozz"
  "${CMAKE_CURRENT_BINARY_DIR}/media/animation2.ozz"
  "${CMAKE_CURRENT_BINARY_DIR}/media/animation3.ozz")

target_link_libraries(sample_select
  sample_framework)
target_copy_shared_libraries(sample_select)

set_target_properties(sample_select
  PROPERTIES FOLDER "samples")

if(EMSCRIPTEN)
  # Resource files are embedded to the output file with emscripten
  set_target_properties(sample_select
    PROPERTIES LINK_FLAGS "--embed-file media --embed-file README.md --memory-init-file 0")

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/sample_select.html
    ${CMAKE_CURRENT_BINARY_DIR}/sample_select.js
    ${CMAKE_CURRENT_BINARY_DIR}/sample_select.wasm
    DESTINATION bin/samples/select)
else()
  install(TARGETS sample_select DESTINATION bin/samples/select)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/media DESTINATION bin/samples/select)
endif(EMSCRIPTEN)

add_test(NAME sample_select COMMAND sample_select "--max_idle_loops=${ozz_sample_testing_loops}" $<$<BOOL:${ozz_run_tests_headless}>:--norender>)
add_test(NAME sample_select_path COMMAND sample_select "--skeleton=media/skeleton.ozz" "--animation1=media/animation1.ozz" "--animation2=media/animation2.ozz"  "--animation3=media/animation3.ozz" "--max_idle_loops=${ozz_sample_testing_loops}" $<$<BOOL:${ozz_run_tests_headless}>:--norender>)
add_test(NAME sample_select_invalid_skeleton_path COMMAND sample_select "--skeleton=media/bad_skeleton.ozz" $<$<BOOL:${ozz_run_tests_headless}>:--norender>)
set_tests_properties(sample_select_invalid_skeleton_path PROPERTIES WILL_FAIL true)
add_test(NAME sample_select_invalid_animation_path1 COMMAND sample_select "--animation1=media/bad_animation.ozz" $<$<BOOL:${ozz_run_tests_headless}>:--norender>)
set_tests_properties(sample_select_invalid_animation_path1 PROPERTIES WILL_FAIL true)
add_test(NAME sample_select_invalid_animation_path2 COMMAND sample_select "--animation2=media/bad_animation.ozz" $<$<BOOL:${ozz_run_tests_headless}>:--norender>)
set_tests_properties(sample_select_invalid_animation_path2 PROPERTIES WILL_FAIL true)
add_test(NAME sample_select_invalid_animation_path3 COMMAND sample_select "--animation3=media/bad_animation.ozz" $<$<BOOL:${ozz_run_tests_headless}>:--norender>)
set_tests_properties(sample_select_invalid_animation_path3 PROPERTIES WILL_FAIL true)
