project(SIMPLE_ROBOT_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${ros_app_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(simple_robot_app fsw/src/simple_robot_app.c)
target_link_libraries(simple_robot_app m)

target_include_directories(simple_robot_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
)
