add_executable(plan_demo plan_demo.cpp)
target_link_libraries(plan_demo PRIVATE mpl)

add_executable(parking_demo parking_demo.cpp)
target_link_libraries(parking_demo PRIVATE mpl)

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PRIVATE mpl)

add_executable(smooth_demo smooth_demo.cpp)
target_link_libraries(smooth_demo PRIVATE mpl)
