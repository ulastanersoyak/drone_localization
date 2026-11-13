add_executable(pole_tracking_sim src/pole_tracking_sim.cpp)
target_link_libraries(pole_tracking_sim PRIVATE pole_estimator xbus
                      gnss_imu_fusion_sim gz-transport13::core
                      gz-msgs10::core)
