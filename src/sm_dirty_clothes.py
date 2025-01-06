import rclpy
from rclpy.executors import MultiThreadedExecutor

import smach_ros
from smach import StateMachine, State
from smach import Concurrence

import yaml
import logging
from typing import Dict, Any

from states_dirty_clothes import (
    Start, Look_For_Clothes, Navigate, 
    QueryUser, End, Teleoperate, Look_For_Dirty,
    Manipulate_from_object_ik, Manipulate_with_joint_values, LookForMarker
)
from stretch_actions_dirty_clothes import StretchActions
from state_controls_dirty_clothes import Controls


def load_config(file_path: str) -> Dict[str, Any]:
    """
    Load and validate the YAML configuration file.
    """
    try:
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
        # Validate required keys
        required_keys = ['nav_initial', 'nav_to_clothes_1', 'nav_to_clothes_2', 'manipulation_pick2', 'nav_laundary_basket', 'manipulation_close_microwave']
        for key in required_keys:
            if key not in config:
                raise KeyError(f"Missing required key in configuration: {key}")
        return config
    except FileNotFoundError:
        raise FileNotFoundError(f"Configuration file not found: {file_path}")
    except yaml.YAMLError as exc:
        raise ValueError(f"Error parsing YAML file: {exc}")


def create_navigation_sub_sm(robot: StretchActions, ui: Controls, config: Dict[str, Any], next_state: str) -> StateMachine:
    """
    Create a reusable sub-state machine for navigation with user intervention.
    """
    sm_nav = StateMachine(outcomes=['success', 'abort', 'preempt'])
    with sm_nav:
        StateMachine.add('NAVIGATE', Navigate(robot, config), 
                         transitions={'success': next_state, 
                                      'fail': 'QUERY_USER', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})
        StateMachine.add('QUERY_USER', QueryUser(ui),
                         transitions={'continue': 'NAVIGATE', 
                                      'teleoperate': 'TELEOPERATE', 
                                      'abort': 'abort', 
                                      'next_state': next_state})
        StateMachine.add('TELEOPERATE', Teleoperate(robot, ui),
                         transitions={'success': 'QUERY_USER', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})
    return sm_nav


def main():
    rclpy.init()
    logger = logging.getLogger('state_machine')
    logger.setLevel(logging.INFO)
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
    logger.addHandler(handler)

    # Load configuration
    try:
        config = load_config('config/sm_microwave.yaml')
    except Exception as e:
        logger.error(f"Failed to load configuration: {e}")
        return

    # Initialize ROS nodes and actions
    executor = MultiThreadedExecutor()
    robot = StretchActions()
    ui = Controls()
    executor.add_node(robot)
    executor.add_node(ui)

    # Main State Machine
    sm = StateMachine(outcomes=['success', 'abort', 'preempt'])
    with sm:
        StateMachine.add('START', Start(robot, config['nav_initial']),
                         transitions={'success': 'NAVIGATE_TO_CLOTHES', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        # Navigation to Clothes
        sm_nav_to_clothes = create_navigation_sub_sm(robot, ui, config['nav_to_clothes_1'], 'LOOK_FOR_CLOTHES')
        StateMachine.add('NAVIGATE_TO_CLOTHES', sm_nav_to_clothes, 
                         transitions={'success': 'LOOK_FOR_CLOTHES', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        StateMachine.add('LOOK_FOR_CLOTHES', Look_For_Clothes(robot, 'cloth', 0.01),
                         transitions={'success': 'DETECT_DIRTY_CLOTHES', 
                                      'fail': 'NAVIGATE_LOCATION_2', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        StateMachine.add('DETECT_DIRTY_CLOTHES', Look_For_Dirty(robot, 'cloth'),
                         transitions={'clean': 'NAVIGATE_LOCATION_2', 
                                      'dirty': 'PICK_DIRTY_CLOTHES', 
                                      'fail': 'NAVIGATE_LOCATION_2', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        # Pick Dirty Clothes with User Intervention
        sm_pick_dirty = create_navigation_sub_sm(robot, ui, config['manipulation_pick2'], 'NAVIGATE_TO_LAUNDRY')
        StateMachine.add('PICK_DIRTY_CLOTHES', sm_pick_dirty, 
                         transitions={'success': 'NAVIGATE_TO_LAUNDRY', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        # Navigation to Laundry
        sm_nav_to_laundry = create_navigation_sub_sm(robot, ui, config['nav_laundary_basket'], 'LOOK_FOR_LAUNDRY')
        StateMachine.add('NAVIGATE_TO_LAUNDRY', sm_nav_to_laundry,
                         transitions={'success': 'LOOK_FOR_LAUNDRY', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        StateMachine.add('LOOK_FOR_LAUNDRY', LookForMarker(robot, 'pick'),
                         transitions={'success': 'DROP_CLOTHES', 
                                      'fail': 'NAVIGATE_TO_LAUNDRY', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        StateMachine.add('DROP_CLOTHES', Manipulate_with_joint_values(robot, config['manipulation_close_microwave']),
                         transitions={'success': 'END', 
                                      'fail': 'DROP_CLOTHES', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

        StateMachine.add('END', End(),
                         transitions={'success': 'success', 
                                      'abort': 'abort', 
                                      'preempt': 'preempt'})

    # Start SMACH Introspection Server
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/state_machine')
    sis.start()

    # Execute the State Machine
    try:
        logger.info("Starting the state machine.")
        sm.execute()
        executor.spin()
    except Exception as e:
        logger.error(f"State machine execution failed: {e}")
    finally:
        logger.info("Shutting down.")
        sis.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
