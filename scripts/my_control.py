#!/usr/bin/env python3


import rospy
from task3_env.srv import *

# toy_type options
GREEN = 'green'
BLUE = 'blue'
BLACK = 'black'
RED = 'red'
TOY_TYPES = [GREEN, BLUE, BLACK, RED]

LOCATION0_TOY_ORDER = [BLUE, BLACK, RED, GREEN]
LOCATION1_TOY_ORDER = [BLACK, RED, BLUE, GREEN]
LOCATION2_TOY_ORDER = [GREEN, BLACK, RED, BLUE]
LOCATION3_TOY_ORDER = [BLACK, RED, BLUE, GREEN]
LOCATIONS_TOY_ORDERS = [LOCATION0_TOY_ORDER, LOCATION1_TOY_ORDER, LOCATION2_TOY_ORDER, LOCATION3_TOY_ORDER]

navigations_left = 8
picks_left = 6
knapsack_toy = None



def call_navigate(location):
    global navigations_left
    print(f"Navigating to: {location}")
    try:
        navigate_srv = rospy.ServiceProxy('navigate', navigate)
        resp = navigate_srv(location)
        print(f"Navigation success (allegedly): {resp.success}")
        navigations_left -= 1
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_pick(toy_type):
    global knapsack_toy
    global picks_left
    try:
        pick_srv = rospy.ServiceProxy('pick', pick)
        resp = pick_srv(toy_type)
        if resp.success:
            knapsack_toy = toy_type
        print(f"Picking {toy_type} (accurate): {resp.success}")
        picks_left -= 1
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_place():
    global knapsack_toy
    try:
        place_srv = rospy.ServiceProxy('place', place)
        resp = place_srv()
        if resp.success:
            knapsack_toy = None
        print(f"Placing (accurate): {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def place_toy_at_baby():
    if not call_navigate(4):
        if navigations_left > 0:
            call_navigate(4)
        else:
            return False
    
    return call_place()

def pick_toy(location, given_toys):
    toy_order = [toy for toy in LOCATIONS_TOY_ORDERS[location] if toy not in given_toys]
    for toy in toy_order:
        if picks_left < 1:
            return None
        if call_pick(toy):
            return toy
    
    return None

def get_and_pick_toy(location, given_toys):
    if navigations_left < 2:
        return None

    if not call_navigate(location):
        # check if there are enough navigations left for navigating again and to the baby
        if navigations_left >= 2 and not call_navigate(location) and navigations_left == 2:
            call_navigate(location)
        
    if navigations_left < 1:
        return None

    return pick_toy(location, given_toys)

def calc_next_location(left_locations):
    if len(left_locations) == 0:
        return None
    
    if len(left_locations) == 2: # assume locations doesn't include 0
        if navigations_left == 2:
            return 2
        else:
            return 1

    return left_locations[0]

def reset_control():
    global knapsack_toy
    global navigations_left
    global picks_left

    navigations_left = 8
    picks_left = 6
    knapsack_toy = None


def run_control():
    reset_control()

    given_toys = []
    next_locations = [2, 3, 1]
    next_location = next_locations[0]
    while next_location is not None: 
        if navigations_left < 2 or picks_left < 1:
            break
        toy = get_and_pick_toy(next_location, given_toys)
        if toy is not None:
            if place_toy_at_baby():
                given_toys.append(toy)
                next_locations.remove(next_location)
        
        next_location = calc_next_location(next_locations)


if __name__ == '__main__':
    run_control()
