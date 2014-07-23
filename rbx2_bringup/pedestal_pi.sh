#!/bin/bash

roslaunch rbx2_bringup dynamixel_a.launch &

sleep 10


roslaunch rbx2_bringup dynamixel_b.launch &

sleep 10

roslaunch rbx2_bringup dynamixel_c.launch
