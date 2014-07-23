cmake_minimum_required(VERSION 2.8.3)

project(rbx2_msgs)

find_package(catkin REQUIRED COMPONENTS	message_generation)
	
add_service_files(DIRECTORY srv
                  FILES
                  SetBatteryLevel.srv)

generate_messages()

catkin_package(DEPENDS CATKIN DEPENDS message_runtime)