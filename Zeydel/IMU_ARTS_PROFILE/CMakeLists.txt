cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(IMU_ARTS_PROFILE)

target_sources(app PRIVATE src/main.c src/imu_data.c src/BNO055_data.c)
