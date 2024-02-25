1. Paste code/my_experiment.py and code/my_control.py into task3_env/scripts
2. Run catkin_make from catkin_ws
3. Run chmod -R +x .
4. Launch (run the first, wait for the simulation to load and then run the second in a new terminal):
roslaunch task3_env task3_env.launch
script -c 'rosrun task3_env my_experiment.py' shell.txt
5. The output files are located in the current folder
