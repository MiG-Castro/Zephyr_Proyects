cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Perfil_IMU_Periferico)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/my_imu_ble_service.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(.)
