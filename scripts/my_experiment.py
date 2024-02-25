#!/usr/bin/env python3

import roslaunch
import rospy
import os
from task3_env.srv import *


# skills_server node global launcher
skills_server_process = None
skills_server_node = roslaunch.core.Node("task3_env", "skills_server.py", name="skills_server_node", output='screen')

control_process = None
control_node = roslaunch.core.Node("task3_env", "my_control.py", name="control_node", output='screen')

# launch.parent.ROSLaunchParent(UUID, filename)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()


def call_navigate(location):
    print(f"Navigating to: {location}")
    try:
        navigate_srv = rospy.ServiceProxy('navigate', navigate)
        resp = navigate_srv(location)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_info():
    try:
        info_srv = rospy.ServiceProxy('info', info)
        resp = info_srv()
        return resp.internal_info
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def relaunch_skills_server():
    # kill the existing skills_server
    global skills_server_process
    skills_server_process.stop() if skills_server_process else os.system("rosnode kill skills_server_node")
    rospy.sleep(3)
    print("### killed skills_server ###")
    
    # launch the skills_server
    skills_server_process = launch.launch(skills_server_node)
    rospy.sleep(5)

    # wait for services launched in server
    print("waiting for services...")
    rospy.wait_for_service('info')
    rospy.wait_for_service('navigate')
    rospy.wait_for_service('pick')
    rospy.wait_for_service('place')
    print("### started skills_server ###")

def reset_env():
    print(f"========== reset env =========")
    # make sure the navigation is running
    relaunch_skills_server()

    # start at the baby
    if not call_navigate(4):
        rospy.sleep(3)  

        # make sure it did not fail
        call_navigate(4)
        rospy.sleep(3)

    # spawn toys and reset counters
    relaunch_skills_server()


def call_control():
    global control_process

    # launch the control
    control_process = launch.launch(control_node)
    while control_process.is_alive():
        rospy.sleep(1)    


################################
def print_infos(infos):
    print("\n\n===============================")
    print(f"==========INFO SUMMARY=========")
    print("===============================")
    for i, info in enumerate(infos):
        print(f"###\nControl {i+1}:\n###\n")
        print(info)
    print("===============================")

# todo validate file path
def export_infos(infos, average_reward):
    with open('experiment_output.txt', 'w') as f:
        for i, info in enumerate(infos):
            f.write(f"=== Info for expirament {i}:\n{info}\n\n")
        f.write(f"=== Average reward: {average_reward}\n\n\n")

def main():
    infos = []
    total_rewards = []
    for i in range(10):
        reset_env()

        print("\n\n===============================")
        print(f"========== CONTROL {i+1} =========")
        print("===============================")
        call_control()
        
        info = call_info()
        infos.append(info)
        total_reward = int(info.split("total rewards:")[1])
        total_rewards.append(total_reward)
        print(f"Total reward: {total_reward}")
        
        print(f"========== Finished {i+1}, Total reward: {total_reward} =========\n\n")
    
    print_infos(infos)
    average_reward = sum(total_rewards) / len(total_rewards)
    print(f"Average reward: {average_reward}")

    export_infos(infos, average_reward)


# todo: launch task3_env from here instead of cli
if __name__ == '__main__':
    try:
        main()
    finally:
        # After Ctrl+C, stop all nodes from running
        if skills_server_process:
            skills_server_process.stop()
        if control_process: 
            control_process.stop()



