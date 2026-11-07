cmake_minimum_required(VERSION 3.16)
project(unitree_ros2_test C CXX)
find_package(ament_cmake REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  include(cmake/dds_peer.cmake)
  find_package(rclcpp REQUIRED)
  add_executable(api_client_test api_client_test.cpp)
  target_compile_features(api_client_test PRIVATE cxx_std_17)
  target_include_directories(api_client_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(api_client_test rclcpp unitree_api)
  ament_add_pytest_test(api_client test_api_client.py TIMEOUT 30
    ENV "UNITREE_API_CLIENT_TEST=$<TARGET_FILE:api_client_test>")
  add_executable(api_client_peer_test api_client_peer_test.cpp)
  target_compile_features(api_client_peer_test PRIVATE cxx_std_17)
  target_include_directories(api_client_peer_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(api_client_peer_test rclcpp unitree_api)
  ament_add_pytest_test(api_client_peer test_api_client_peer.py TIMEOUT 60
    ENV "UNITREE_DDS_API_SERVER=$<TARGET_FILE:dds_api_server>"
    "UNITREE_API_CLIENT_PEER_TEST=$<TARGET_FILE:api_client_peer_test>")
  add_executable(b2_api_client_test b2_api_client_test.cpp)
  target_compile_features(b2_api_client_test PRIVATE cxx_std_17)
  target_include_directories(b2_api_client_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(b2_api_client_test rclcpp unitree_api)
  ament_add_pytest_test(b2_api_client test_b2_api_client.py TIMEOUT 30
    ENV "UNITREE_B2_API_CLIENT_TEST=$<TARGET_FILE:b2_api_client_test>")
  ament_add_pytest_test(replay_tool test_replay_tool.py TIMEOUT 60)
  ament_add_pytest_test(bag_tool test_bag_tool.py TIMEOUT 60)
  ament_add_pytest_test(independent_bags test_compare_bags.py TIMEOUT 60)
  ament_add_pytest_test(interfaces test_interfaces.py TIMEOUT 120)
  ament_add_pytest_test(native_dds test_native_dds.py TIMEOUT 120
    ENV "UNITREE_DDS_PEER=$<TARGET_FILE:dds_peer>")
  ament_add_pytest_test(examples test_examples.py TIMEOUT 120)
  ament_add_pytest_test(setup_scripts test_setup.py TIMEOUT 30)
endif()
ament_package()
