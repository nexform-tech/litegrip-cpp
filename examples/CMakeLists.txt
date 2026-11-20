# Examples are plain C++ programs that link the SDK. They are the "plain C++
# consumer" proof: nothing here may include anything beyond the SDK and the
# standard library.

set(LITEGRIP_CPP_EXAMPLES
  version_info.cpp
  gripper_example.cpp
  open_close.cpp
  move_to.cpp
  teleop_master.cpp
  teleop_slave.cpp
)

foreach(example_src IN LISTS LITEGRIP_CPP_EXAMPLES)
  get_filename_component(example_name ${example_src} NAME_WE)
  add_executable(litegrip_example_${example_name} ${example_src})
  target_link_libraries(litegrip_example_${example_name} PRIVATE litegrip_cpp::litegrip_cpp)
endforeach()
