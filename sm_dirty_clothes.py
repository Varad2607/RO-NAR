from typing import Optional
import rclpy
from rclpy.executors import MultiThreadedExecutor

import smach_ros
from smach import StateMachine, State

from smach import StateMachine as SM

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
        StateMachine.add('START', Start(robot, config['nav_initial']), transitions={'success':'NAVIGATE_TO_CLOTHES', 'abort':'abort', 'preempt':'preempt', 'fail':'abort'})
        StateMachine.add('NAVIGATE_TO_CLOTHES', Navigate(robot, config['nav_to_clothes_1']), transitions={'success':'LOOK_FOR_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_TO_CLOTHES_QUERY_USER'})
        StateMachine.add('NAVIGATE_TO_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'START', 'continue':'NAVIGATE_TO_CLOTHES', 'next_state':'LOOK_FOR_CLOTHES', 'teleoperate':'NAVIGATE_TO_CLOTHES_TELEOPERATE' })
        StateMachine.add('NAVIGATE_TO_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_TO_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        StateMachine.add('LOOK_FOR_CLOTHES', Look_For_Clothes(robot, 'cloth',0.01), transitions={'success':'DETECT_DIRTY_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_LOCATION_2'})

        StateMachine.add('DETECT_DIRTY_CLOTHES', Look_For_Dirty(robot, 'cloth'), transitions={'clean':'NAVIGATE_LOCATION_2', 'dirty':'PICK_DIRTY_CLOTHES', 'abort':'abort','fail':'NAVIGATE_LOCATION_2','preempt':'preempt'})

        StateMachine.add('PICK_DIRTY_CLOTHES', Manipulate_from_object_ik(robot, 'cloth', config['manipulation_pick2']), transitions={'success':'NAVIGATE_TO_LAUNDRY', 'preempt':'preempt', 'abort':'abort', 'fail':'PICK_DIRTY_CLOTHES_QUERY_USER'})
        StateMachine.add('PICK_DIRTY_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'LOOK_FOR_CLOTHES', 'continue':'PICK_DIRTY_CLOTHES', 'next_state':'NAVIGATE_TO_LAUNDRY', 'teleoperate':'PICK_DIRTY_CLOTHES_TELEOPERATE' })
        StateMachine.add('PICK_DIRTY_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'PICK_DIRTY_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        StateMachine.add('NAVIGATE_TO_LAUNDRY', Navigate(robot, config['nav_laundary_basket']), transitions={'success':'LOOK_FOR_LAUNDRY', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_TO_LAUNDRY_QUERY_USER'})
        StateMachine.add('NAVIGATE_TO_LAUNDRY_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'PICK_DIRTY_CLOTHES', 'continue':'NAVIGATE_TO_LAUNDRY', 'next_state':'LOOK_FOR_LAUNDRY', 'teleoperate':'NAVIGATE_TO_LAUNDRY_TELEOPERATE' })
        StateMachine.add('NAVIGATE_TO_LAUNDRY_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_TO_LAUNDRY_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        StateMachine.add('LOOK_FOR_LAUNDRY', LookForMarker(robot, 'pick'), transitions={'success':'DROP_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'LOOK_FOR_LAUNDRY_QUERY_USER'})
        StateMachine.add('LOOK_FOR_LAUNDRY_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'NAVIGATE_TO_LAUNDRY', 'continue':'LOOK_FOR_LAUNDRY', 'next_state':'DROP_CLOTHES', 'teleoperate':'LOOK_FOR_LAUNDRY_TELEOPERATE' })
        StateMachine.add('LOOK_FOR_LAUNDRY_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'LOOK_FOR_LAUNDRY_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

        StateMachine.add('DROP_CLOTHES', Manipulate_with_joint_values(robot, config['manipulation_close_microwave']), transitions={'success':'END', 'preempt':'preempt', 'abort':'abort', 'fail':'DROP_CLOTHES_QUERY_USER'})
        StateMachine.add('DROP_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'LOOK_FOR_LAUNDRY', 'continue':'DROP_CLOTHES', 'next_state':'END', 'teleoperate':'DROP_CLOTHES_TELEOPERATE'})
        StateMachine.add('DROP_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'DROP_CLOTHES_QUERY_USER','abort':'abort', 'preempt':'preempt'})
       
        StateMachine.add('NAVIGATE_LOCATION_2', Navigate(robot, config['nav_to_clothes_2']), transitions={'success':'END', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_LOCATION_2_QUERY_USER'})
        StateMachine.add('NAVIGATE_LOCATION_2_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'DROP_CLOTHES', 'continue':'NAVIGATE_LOCATION_2', 'next_state':'END', 'teleoperate':'NAVIGATE_LOCATION_2_TELEOPERATE'})
        StateMachine.add('NAVIGATE_LOCATION_2_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_LOCATION_2_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})
        
        StateMachine.add('END', End(), transitions={'success':'success', 'abort':'abort', 'preempt':'preempt', 'fail':'abort'})
   
   
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










# class Counter(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['continue', 'done'])
#         self.counter = 0
#         self.max_count = 3

#     def execute(self, userdata):
#         self.counter += 1
#         if self.counter < self.max_count:
#             return 'continue'
#         else:
#             return 'done'


# class IterateList(State):
#     def __init__(self, key: str, outkey:Optional[str]=None):
#         outputs = ["index"]
#         self.outkey = outkey
#         if outkey:
#             outputs.append(outkey)
#         State.__init__(self, outcomes=['repeat', 'done'], input_keys=[key], output_keys=outputs)
#         self.counter = -1
#         self.key = key

#     def execute(self, userdata):
#         self.counter = self.counter + 1
#         userdata["index"] = self.counter

#         if self.counter >= len(userdata[self.key]):
#             self.counter = 0
#             userdata["index"] = self.counter
#             return 'done'
#         else:
#             if self.outkey:
#                 userdata[self.outkey] = userdata[self.key][self.counter]
            # return 'repeat'

# def inject_userdata_auto(name, output_key, value):
#     class InjectUserdata(State):
#         def __init__(self, ):
#             State.__init__(self, outcomes=['succeeded'], output_keys=[output_key])
#             self.output_key = output_key

#         def execute(self, userdata):
#             userdata[output_key] = value
#             return 'succeeded'

#     StateMachine.add_auto(name, InjectUserdata(), ["succeeded"])


# def remap_auto(name, from_key, to_key):
#     class Remap(State):
#         def __init__(self):
#             State.__init__(self, outcomes=["succeeded"], input_keys=[from_key], output_keys=[to_key])

#         def execute(self, ud):
#             ud[to_key] = ud[from_key]
#             return "succeeded"

#     smach.StateMachine.add_auto(name, Remap(), ["succeeded"])


# start_sm =SM(outcomes=['success', 'abort','preempt'])

    # tidy_up_clothes_sm = StateMachine(outcomes=['success', 'abort', 'preempt'], input_keys=['location'])

    # with tidy_up_clothes_sm:
    #     # For testing purpsoses: load one location manually. We expect the outer state machine to do this
    #     # inject_userdata_auto("LOAD_LOCATIONS", "location", config['locations'][0])
    #     remap_auto("REMAP_LOCATION", "location", "pose")
    #     StateMachine.add('NAVIGATE_TO_CLOTHES', Navigate(robot), transitions={'success':'LOOK_FOR_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_TO_CLOTHES_QUERY_USER'})
    #     StateMachine.add('NAVIGATE_TO_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'START', 'continue':'NAVIGATE_TO_CLOTHES', 'next_state':'LOOK_FOR_CLOTHES', 'teleoperate':'NAVIGATE_TO_CLOTHES_TELEOPERATE' })
    #     StateMachine.add('NAVIGATE_TO_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_TO_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

    #     StateMachine.add('LOOK_FOR_CLOTHES', Look_For_Clothes(robot, 'cloth',0.01), transitions={'success':'DETECT_DIRTY_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_LOCATION_2'})

    #     StateMachine.add('DETECT_DIRTY_CLOTHES', Look_For_Dirty(robot, 'cloth'), transitions={'clean':'NAVIGATE_LOCATION_2', 'dirty':'PICK_DIRTY_CLOTHES', 'abort':'abort','fail':'NAVIGATE_LOCATION_2','preempt':'preempt'})

    #     StateMachine.add('PICK_DIRTY_CLOTHES', Manipulate_from_object_ik(robot, 'clothes', config['manipulation_open_microwave']), transitions={'success':'NAVIGATE_TO_LAUNDRY', 'preempt':'preempt', 'abort':'abort', 'fail':'PICK_DIRTY_CLOTHES_QUERY_USER'})
    #     StateMachine.add('PICK_DIRTY_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'LOOK_FOR_CLOTHES', 'continue':'PICK_DIRTY_CLOTHES', 'next_state':'NAVIGATE_TO_LAUNDRY', 'teleoperate':'PICK_DIRTY_CLOTHES_TELEOPERATE' })
    #     StateMachine.add('PICK_DIRTY_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'PICK_DIRTY_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

    #     StateMachine.add('NAVIGATE_TO_LAUNDRY', Navigate(robot, config['nav_laundary_basket']), transitions={'success':'LOOK_FOR_LAUNDRY', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_TO_LAUNDRY_QUERY_USER'})
    #     StateMachine.add('NAVIGATE_TO_LAUNDRY_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'PICK_DIRTY_CLOTHES', 'continue':'NAVIGATE_TO_LAUNDRY', 'next_state':'LOOK_FOR_LAUNDRY', 'teleoperate':'NAVIGATE_TO_LAUNDRY_TELEOPERATE' })
    #     StateMachine.add('NAVIGATE_TO_LAUNDRY_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_TO_LAUNDRY_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

    #     StateMachine.add('LOOK_FOR_LAUNDRY', LookForMarker(robot, 'pick'), transitions={'success':'DROP_CLOTHES', 'preempt':'preempt', 'abort':'abort', 'fail':'LOOK_FOR_LAUNDRY_QUERY_USER'})
    #     StateMachine.add('LOOK_FOR_LAUNDRY_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'NAVIGATE_TO_LAUNDRY', 'continue':'LOOK_FOR_LAUNDRY', 'next_state':'DROP_CLOTHES', 'teleoperate':'LOOK_FOR_LAUNDRY_TELEOPERATE' })
    #     StateMachine.add('LOOK_FOR_LAUNDRY_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'LOOK_FOR_LAUNDRY_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

    #     StateMachine.add('DROP_CLOTHES', Manipulate_with_joint_values(robot, config['manipulation_pick']), transitions={'success':'END', 'preempt':'preempt', 'abort':'abort', 'fail':'DROP_CLOTHES_QUERY_USER'})
    #     StateMachine.add('DROP_CLOTHES_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'LOOK_FOR_LAUNDRY', 'continue':'DROP_CLOTHES', 'next_state':'END', 'teleoperate':'DROP_CLOTHES_TELEOPERATE' })
    #     StateMachine.add('DROP_CLOTHES_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'DROP_CLOTHES_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})


    # with start_sm:
        
    #     SM.add('START', Start(robot, config['nav_initial']), transitions={'success':'NAVIGATE_TO_CLOTHES', 'abort':'abort', 'preempt':'preempt', 'fail':'abort'})
    #     inject_userdata_auto("LOAD_LOCATIONS", "locations", config['locations'])
    #     SM.add("ITERATE", IterateList("locations", "location"), transitions={"repeat": "TIDY_UP_IN_LOCATION", "done": "success"})
    #     SM.add('TIDY_UP_IN_LOCATION', tidy_up_clothes_sm, transitions={'success':'success', 'abort':'abort', 'preempt':'preempt'})


    #     StateMachine.add('NAVIGATE_LOCATION_2', Navigate(robot, config['nav_to_clothes_2']), transitions={'success':'END', 'preempt':'preempt', 'abort':'abort', 'fail':'NAVIGATE_LOCATION_2_QUERY_USER'})
    #     StateMachine.add('NAVIGATE_LOCATION_2_QUERY_USER', QueryUser(ui), transitions={'abort':'abort', 'previous_state':'DROP_CLOTHES', 'continue':'NAVIGATE_LOCATION_2', 'next_state':'END', 'teleoperate':'NAVIGATE_LOCATION_2_TELEOPERATE' })
    #     StateMachine.add('NAVIGATE_LOCATION_2_TELEOPERATE', Teleoperate(robot, ui), transitions={'success':'NAVIGATE_LOCATION_2_QUERY_USER', 'abort':'abort', 'preempt':'preempt'})

    #     StateMachine.add('END', End(), transitions={'success':'success', 'abort':'abort', 'preempt':'preempt', 'fail':'abort'})