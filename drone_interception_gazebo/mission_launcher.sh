#!/bin/bash


echo "Sarting mission1.py"
rosservice call /drone111/python_based_mission_interpreter_process/start "{}" 
sleep 1;
echo "Sarting mission2.py"
rosservice call /drone112/python_based_mission_interpreter_process/start "{}" 

