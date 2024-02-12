#!/usr/bin/env python3

import rospy
import roslaunch
import os
from task3_env.srv import *

# toy_type options
GREEN = 'green'
BLUE = 'blue'
BLACK = 'black'
RED = 'red'
TOY_TYPES = [GREEN, BLUE, BLACK, RED]

# skills_server node global launcher
process = None
node = roslaunch.core.Node("task3_env", "skills_server.py", name="skills_server_node")
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()


def call_navigate(location):
    print(f"Navigating to: {location}")
    try:
        navigate_srv = rospy.ServiceProxy('navigate', navigate)
        resp = navigate_srv(location)
        print(f"Navigation success (allegedly): {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def call_pick(toy_type):
    try:
        pick_srv = rospy.ServiceProxy('pick', pick)
        resp = pick_srv(toy_type)
        print(f"Picking {toy_type} (accurate): {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def call_place():
    try:
        place_srv = rospy.ServiceProxy('place', place)
        resp = place_srv()
        print(f"Placing (accurate): {resp.success}")
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
    global process
    process.stop() if process else os.system("rosnode kill skills_server_node")
    rospy.sleep(3)
    print("### killed skills_server ###")
    
    # launch the skills_server
    process = launch.launch(node)
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
        # make sure it did not fail
        call_navigate(4)

    # spawn toys and reset counters
    relaunch_skills_server()


# todo: move implementation to control script
def run_control():
    # navigate to location 0
    navigation_success = call_navigate(0)
    while not navigation_success:
        navigation_success = call_navigate(0)
    
    # pick the toy
    for toy_type in TOY_TYPES:
        pick_success = call_pick(toy_type)
        if pick_success:
            break
    
    # navigate to location 4 (baby)
    navigation_success = call_navigate(4)
    while not navigation_success:
        navigation_success = call_navigate(4)
    
    # place the toy
    if pick_success and navigation_success:
        call_place()


def print_infos(infos):
    print("\n\n===============================")
    print(f"==========INFO SUMMARY=========")
    print("===============================")
    for i, info in enumerate(infos):
        print(f"Control {i+1}: {info}")
    print("===============================")


def main():
    infos = []
    for i in range(3):
        reset_env()
        print("\n\n===============================")
        print(f"========== CONTROL {i+1} =========")
        print("===============================")
        run_control()
        infos.append(call_info())
        print(f"========== Finished {i+1} =========\n\n")
    
    print_infos(infos)


# todo: launch task3_env from here instead of cli
if __name__ == '__main__':
    try:
        main()
    finally:
        # After Ctrl+C, stop all nodes from running
        if process:
            process.stop()



