cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(base_node)

list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

# Compile main.c **and** the Nanopb source that defines SensorData_msg
target_sources(app PRIVATE
    src/main.c
    src/sensor_packet.pb.c
)