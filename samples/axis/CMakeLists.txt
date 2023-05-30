add_custom_command(
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/README.md"
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/README.md"
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/README.md .
  VERBATIM)

add_executable(sample_axis
  sample_axis.cc
  ${CMAKE_CURRENT_BINARY_DIR}/README.md)
target_link_libraries(sample_axis
  ozz_animation_offline
  sample_framework)
target_copy_shared_libraries(sample_axis)

set_target_properties(sample_axis
  PROPERTIES FOLDER "samples")

if(EMSCRIPTEN)
  set_target_properties(sample_axis
    PROPERTIES LINK_FLAGS "--embed-file README.md --memory-init-file 0")

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/sample_axis.html
    ${CMAKE_CURRENT_BINARY_DIR}/sample_axis.js
    ${CMAKE_CURRENT_BINARY_DIR}/sample_axis.wasm
    DESTINATION bin/samples/axis)
else()
  install(TARGETS sample_axis DESTINATION bin/samples/axis)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/README.md DESTINATION bin/samples/axis)
endif(EMSCRIPTEN)

add_test(NAME sample_axis COMMAND sample_axis "--max_idle_loops=${ozz_sample_testing_loops}" $<$<BOOL:${ozz_run_tests_headless}>:--norender>)
