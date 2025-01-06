import rclpy
from rclpy.executors import MultiThreadedExecutor

import smach_ros
from smach import StateMachine, State

import yaml
import threading
import logging

from states_dirty_clothes import (
    Start, Look_For_Clothes, Navigate, 
    QueryUser, End, Teleoperate, Look_For_Dirty,Manipulate_from_object_ik, Manipulate_with_joint_values, LookForMarker
)

from stretch_actions_dirty_clothes import StretchActions
from state_controls_dirty_clothes import Controls

def main():

    try:
        with open('config/sm_microwave.yaml', 'r') as file:
            config =yaml.safe_load(file)

    except FileNotFoundError:
        print("Configuration file not found.")
        return
    
    except yaml.YAMLError as exc:
        print(f"Error in configuration file: {exc}")
        return
    

    rclpy.init()
    logger = logging.getLogger(__name__)

    executor = MultiThreadedExecutor()
    robot = StretchActions()
    ui= Controls()
    executor.add_node(robot)
    executor.add_node(ui)


    sm=StateMachine(outcomes=['success', 'abort','preempt'])

    with sm:
        StateMachine.add('PICK_DIRTY_CLOTHES', Manipulate_from_object_ik(robot, 'cloth', config['manipulation_pick2']), transitions={'success':'success', 'preempt':'preempt', 'abort':'abort', 'fail':'abort'})


        # StateMachine.add('LOOK_FOR_CLOTHES', Look_For_Clothes(robot, 'clothes'), transitions={'success':'DETECT_DIRTY_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_LOCATION_2'})
        # StateMachine.add('START', Start(robot, config['nav_initial']), transitions={'success':'NAVIGATE_TO_CLOTHES', 'abort':'abort', 'preempt':'preempt', 'fail':'abort'})

        # StateMachine.add('NAVIGATE_TO_CLOTHES', Navigate(robot, config['nav_open_microwave']), transitions={'success':'LOOK_FOR_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_TO_CLOTHES_QUERY_USER'})
        # StateMachine.add('NAVIGATE_TO_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'START', 'continue':'NAVIGATE_TO_CLOTHES', 'next_state':'LOOK_FOR_CLOTHES', 'teleoperate':'NAVIGATE_TO_CLOTHES_TELEOPERATE' })
        # StateMachine.add('NAVIGATE_TO_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_TO_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        # StateMachine.add('LOOK_FOR_CLOTHES', Look_For_Clothes(robot, 'cloth', 0.01), transitions={'success':'DETECT_DIRTY_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'abort'})



        # StateMachine.add('DETECT_DIRTY_CLOTHES', Look_For_Dirty(robot, 'cloth'), transitions={'clean':'success', 'dirty':'preempt', 'abort':'abort','fail':'abort','preempt':'preempt'})

        # StateMachine.add('DECISION_AFTER_DETETCTION', After_detetction(robot,after_detection), transitions={'clean':'NAVIGATE_LOCATION_2', 'dirty':'PICKUP_DIRTY_CLOTHES', 'abort':'abort', 'fail':'DETECT_DIRTY_CLOTHES_QUERY_USER'})

        # StateMachine.add('PICK_DIRTY_CLOTHES', Manipulate_from_object_ik(robot, 'clothes', config['manipulation_open_microwave']), transitions={'success':'NAVIGATE_TO_LAUNDRY', 'preempt':'preempt', 'abort':'abort', 'fail':'PICK_DIRTY_CLOTHES_QUERY_USER'})
        # StateMachine.add('PICK_DIRTY_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'LOOK_FOR_CLOTHES', 'continue':'PICK_DIRTY_CLOTHES', 'next_state':'NAVIGATE_TO_LAUNDRY', 'teleoperate':'PICK_DIRTY_CLOTHES_TELEOPERATE' })
        # StateMachine.add('PICK_DIRTY_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'PICK_DIRTY_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        # StateMachine.add('NAVIGATE_TO_LAUNDRY', Navigate(robot, config['nav_pick']), transitions={'success':'LOOK_FOR_LAUNDRY', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_TO_LAUNDRY_QUERY_USER'})
        # StateMachine.add('NAVIGATE_TO_LAUNDRY_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'PICK_DIRTY_CLOTHES', 'continue':'NAVIGATE_TO_LAUNDRY', 'next_state':'LOOK_FOR_LAUNDRY', 'teleoperate':'NAVIGATE_TO_LAUNDRY_TELEOPERATE' })
        # StateMachine.add('NAVIGATE_TO_LAUNDRY_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_TO_LAUNDRY_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        # StateMachine.add('LOOK_FOR_LAUNDRY', LookForMarker(robot, 'pick'), transitions={'success':'DROP_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'LOOK_FOR_LAUNDRY_QUERY_USER'})
        # StateMachine.add('LOOK_FOR_LAUNDRY_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'NAVIGATE_TO_LAUNDRY', 'continue':'LOOK_FOR_LAUNDRY', 'next_state':'DROP_CLOTHES', 'teleoperate':'LOOK_FOR_LAUNDRY_TELEOPERATE' })
        # StateMachine.add('LOOK_FOR_LAUNDRY_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'LOOK_FOR_LAUNDRY_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        # StateMachine.add('DROP_CLOTHES', Manipulate_with_joint_values(robot, config['manipulation_pick']), transitions={'success':'END', 'preempt':'preempt', 'abort':'abort', 'fail':'DROP_CLOTHES_QUERY_USER'})
        # StateMachine.add('DROP_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'LOOK_FOR_LAUNDRY', 'continue':'DROP_CLOTHES', 'next_state':'END', 'teleoperate':'DROP_CLOTHES_TELEOPERATE' })
        # StateMachine.add('DROP_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'DROP_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})


        # StateMachine.add('NAVIGATE_LOCATION_2', Navigate(robot, config['nav_pick']), transitions={'success':'END', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_LOCATION_2_QUERY_USER'})
        # StateMachine.add('NAVIGATE_LOCATION_2_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'DROP_CLOTHES', 'continue':'NAVIGATE_LOCATION_2', 'next_state':'END', 'teleoperate':'NAVIGATE_LOCATION_2_TELEOPERATE' })
        # StateMachine.add('NAVIGATE_LOCATION_2_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_LOCATION_2_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        # StateMachine.add('END', End(), transitions={'success':'success', 'abort':'abort', 'preempt':'preempt', 'fail':'abort'})
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/state_machine')
    sis.start()

    try:
        threading.Thread(target=sm.execute).start()
        executor.spin()

    except Exception as e:
        logger.error(f"State machine execution failed: {e}")

    finally:
        sis.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




