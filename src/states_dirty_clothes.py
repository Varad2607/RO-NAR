from smach import State
from stretch_actions_dirty_clothes import StretchActions
from state_controls_dirty_clothes import Controls


class Start(State):
    def __init__(self,robot:StretchActions, pose):
        State.__init__(self, outcomes=['success', 'abort','preempt','fail'])
        self.robot=robot
        self.pose=pose

    def execute(self, userdata):
        transition = self.robot.publish_initial_pose(self.pose)

        return transition
    

class End(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'abort','preempt','fail'])

    def execute(self, userdata):
        return 'success'
    


class Navigate(State):
    def __init__(self,robot:StretchActions, pose=None):
        State.__init__(self,outcomes=['success','fail','abort','preempt'], input_keys=['pose'])
        self.robot=robot
        self.pose=pose
    

    def execute(self, userdata):
        self.robot.navigation_camera()
        if self.pose is None:
            transition = self.robot.navigate_to_pose(userdata['pose'])
        else:
            transition = self.robot.navigate_to_pose(self.pose)

        return transition
        

class Look_For_Clothes(State):
    def __init__(self,robot:StretchActions, target_object, confidence=0.01, head_tilt=-0.5, pan_steps=9, left_limits=-3.6, right_limits=1.45):
       State.__init__(self,outcomes=['success','fail','abort','preempt'])
       self.robot=robot
       self.target_object=target_object
       self.confidence=confidence
       self.head_tilt=head_tilt
       self.pan_steps=pan_steps
       self.left_limits=left_limits
       self.right_limits=right_limits

    def execute(self, userdata):
        self.robot.manipulation_camera()
        transition = self.robot.look_for_object(self.target_object, self.confidence, self.head_tilt, self.pan_steps, self.left_limits, self.right_limits)
        return transition
    
class Look_For_Dirty(State):
    def __init__(self,robot:StretchActions,target_object,confidence=0.5,head_tilt=-0.5,pan_steps=9,left_limits=-3.6,right_limits=1.45):
        State.__init__(self,outcomes=['clean','dirty','abort','preempt','fail'])
        self.robot=robot
        self.target_object=target_object
        self.confidence=confidence
        self.head_tilt=head_tilt
        self.pan_steps=pan_steps
        self.left_limits=left_limits
        self.right_limits=right_limits
        self.data=None


    def execute(self,userdata):
        self.robot.manipulation_camera()
        print("Looking for dirty clothes")
        transition=self.robot.look_for_dirty_clothes(self.target_object,self.confidence,self.head_tilt,self.pan_steps,self.left_limits,self.right_limits)
        return transition
    


class Manipulate_from_object_ik(State):
    def __init__(self,robot:StretchActions,target_object,manipulation_info):
        State.__init__(self,outcomes=['success','fail','abort','preempt'])
        self.robot=robot
        self.target_object=target_object
        self.manipulation_info=manipulation_info

    def execute(self, userdata):
        transition = self.robot.manipulate_from_object_ik(self.target_object, self.manipulation_info)
        return transition
    


class Manipulate_with_joint_values(State):
    def __init__(self,robot:StretchActions, joint_values):
        State.__init__(self,outcomes=['success','fail','abort','preempt'])
        self.robot=robot
        self.joint_values=joint_values


    def execute(self,userdata):
        transition=self.robot.follow_joint_trajectory(self.joint_values)

        return transition


class QueryUser(State):
    def __init__(self,ui:Controls):
        State.__init__(self,outcomes=['abort','continue','next_state','teleoperate','previous_state'])
        self.ui=ui

    def execute(self,userdata):
        transition = self.ui.query_user()
        return transition
    

class Teleoperate(State):
    def __init__(self,robot:StretchActions,ui:Controls):
        State.__init__(self,outcomes=['success','abort','preempt'])
        self.robot=robot
        self.ui=ui
    
    def execute(self,userdata):
        self.robot.switch_to_position_mode()
        transition = self.ui.query_user_teleop()

        return transition
    

class LookForMarker(State):
    def __init__(self, robot: StretchActions, marker_name, head_tilt=0.0, pan_steps=9, left_limit=-3.6, right_limit=1.45):
        State.__init__(self, outcomes=['success', 'fail', 'abort', 'preempt'])
        self.robot = robot
        self.marker_name = marker_name
        self.head_tilt = head_tilt
        self.pan_steps = pan_steps
        self.left_limit = left_limit
        self.right_limit = right_limit

    def execute(self, userdata):
        transition = self.robot.look_for_marker(self.marker_name, self.head_tilt, self.pan_steps, self.left_limit, self.right_limit)

        return transition



       
       
    

        