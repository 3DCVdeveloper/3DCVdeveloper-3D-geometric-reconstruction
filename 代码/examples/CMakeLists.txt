add_executable(run_cam run_cam.cpp)
target_link_libraries(run_cam myorbbec)

add_executable(reconstruction reconstruction.cpp)
target_link_libraries(reconstruction myorbbec)