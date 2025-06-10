cmake_minimum_required(VERSION 3.5)

project(gopro_webcam_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

include_directories(include)
# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_include_directories("include/${PROJECT_NAME}")

##
## Demos
##

add_library(gopro_webcam_lib
src/image_pipeline/webcam.cpp
)

# Add the OpenCV include directories to the gopro_webcam_lib target.
target_include_directories(gopro_webcam_lib PRIVATE 
${OpenCV_INCLUDE_DIRS}
${CURL_INCLUDE_DIRS}
)

# A single program with one of each of the image pipeline demo nodes.
add_executable(image_pipeline_all_in_one
  src/image_pipeline/image_pipeline_all_in_one.cpp)
target_link_libraries(image_pipeline_all_in_one
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui
  gopro_webcam_lib
  ${CURL_LIBRARIES}
  )

# A single program with one of each of the image pipeline demo nodes, but two image views.
add_executable(image_pipeline_with_two_image_view
  src/image_pipeline/image_pipeline_with_two_image_view.cpp)
target_link_libraries(image_pipeline_with_two_image_view
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui
  gopro_webcam_lib
  ${CURL_LIBRARIES}
  )


  # A stand alone node which produces images from a camera using OpenCV.
  add_executable(camera_node
    src/image_pipeline/camera_node.cpp)

  target_link_libraries(camera_node
    PRIVATE gopro_webcam_lib
    rclcpp::rclcpp
    ${builtin_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    ${CURL_LIBRARIES}
    ${OpenCV_LIBS}  
    )

# A stand alone node which consumes images and displays them using OpenCV.
add_executable(image_view_node
  src/image_pipeline/image_view_node.cpp)
target_link_libraries(image_view_node
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui
  gopro_webcam_lib
  ${CURL_LIBRARIES})

install(TARGETS
  image_pipeline_all_in_one
  image_pipeline_with_two_image_view
  camera_node
  image_view_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_pytest REQUIRED)
#   find_package(launch_testing_ament_cmake REQUIRED)
#   find_package(rmw_implementation_cmake REQUIRED)

#   # Add each test case.  Multi-executable tests can be specified in
#   # semicolon-separated strings, like  exe1:exe2.
#   set(demo_tests
#     cyclic_pipeline
#     two_node_pipeline)

#   macro(tests)
#     foreach(demo_test ${demo_tests})
#       string(REPLACE ":" ";" demo_executables "${demo_test}")
#       set(RCLCPP_DEMOS_EXPECTED_OUTPUT "")
#       foreach(executable ${demo_executables})
#         list(APPEND RCLCPP_DEMOS_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
#       endforeach()

#       set(RCLCPP_DEMOS_EXECUTABLE "")
#       foreach(executable ${demo_executables})
#         list(APPEND RCLCPP_DEMOS_EXECUTABLE "$<TARGET_FILE:${executable}>")
#       endforeach()

#       string(REPLACE ";" "_" exe_list_underscore "${demo_executables}")
#       configure_file(
#         test/test_executables_demo.py.in
#         test_${exe_list_underscore}${target_suffix}.py.configured
#         @ONLY
#       )
#       file(GENERATE
#         OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
#         INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}.py.configured"
#       )

#       add_launch_test(
#         "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
#         TARGET test_demo_${exe_list_underscore}${target_suffix}
#         TIMEOUT 30
#         ENV
#         RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
#         RMW_IMPLEMENTATION=${rmw_implementation}
#       )
#       if(TEST test_demo_${exe_list_underscore}${target_suffix})
#         set_tests_properties(test_demo_${exe_list_underscore}${target_suffix}
#           PROPERTIES DEPENDS "test_demo_${exe_list_underscore}${target_suffix}  test_demo_${exe_list_underscore}${target_suffix}")
#       endif()
#     endforeach()
#   endmacro()

#   call_for_each_rmw_implementation(tests)
# endif()

ament_package()
