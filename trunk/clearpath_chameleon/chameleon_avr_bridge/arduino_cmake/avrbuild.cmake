cmake_minimum_required(VERSION 2.6)


set(ARDUINO_CORE_DIR  "${chameleon_avr_bridge_PACKAGE_PATH}/arduino_cmake/arduino")
include( "${chameleon_avr_bridge_PACKAGE_PATH}/arduino_cmake/cmake_scripts/${ARDUINO_TYPE}.cmake" )

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("/usr/share/arduino/libraries/Wire")


add_custom_target(gen_avr_ros ALL)
add_custom_command(TARGET gen_avr_ros PRE_BUILD
        COMMAND rosrun chameleon_avr_bridge gen_avr.py ${PROJECT_SOURCE_DIR}/${AVR_BRIDGE_CONFIG} ${PROJECT_SOURCE_DIR}/src )
       
execute_process(       COMMAND rosrun chameleon_avr_bridge gen_avr.py ${PROJECT_SOURCE_DIR}/${AVR_BRIDGE_CONFIG} ${PROJECT_SOURCE_DIR}/src )

file(GLOB AVR_ROS_SRC
    "src/avr_ros/*.cpp"
)

#to compile, use make
#to program arduino on /dev/ttyUSB0, do make flash
#to check the program size use make size