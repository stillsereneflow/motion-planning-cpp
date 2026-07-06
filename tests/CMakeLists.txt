find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip)
  # Prevent GoogleTest from overriding the parent project's compiler settings on Windows.
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

add_executable(mpl_tests
  test_astar.cpp
  test_distance_field.cpp
  test_grid_map.cpp
  test_reeds_shepp.cpp
  test_vehicle.cpp)
target_link_libraries(mpl_tests PRIVATE mpl GTest::gtest_main)

gtest_discover_tests(mpl_tests)
