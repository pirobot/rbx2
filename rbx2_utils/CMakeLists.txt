cmake_minimum_required(VERSION 2.8.3)

project(rbx2_utils)

find_package(catkin REQUIRED COMPONENTS	dynamic_reconfigure message_generation)

add_service_files(DIRECTORY srv
                  FILES
                  KillProcess.srv
                  LaunchProcess.srv)
                  
generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
	cfg/BatterySimulator.cfg
	cfg/Pub3DTarget.cfg
	)

catkin_package(DEPENDS CATKIN DEPENDS dynamic_reconfigure message_runtime)
