import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from robot_interfaces.srv import ServiceString

from threading import Event


class Controls(Node):
    def __init__(self):
        super().__init__('controls')
        self.callback_group = ReentrantCallbackGroup()

        self.user_descision = None
        self.user_decision_made=Event()

        self.user_decision_service = self.create_service(ServiceString, '/user_decision_service', self.user_decision_service_callback, callback_group=self.callback_group)
    

    def user_decision_service_callback(self, request, response):

        self.user_decision = request.data
        response.result = f'user decision: {request.data}'
        self.user_decision_made.set()
        return response
    
    def query_user(self):
        self.user_decision = None
        self.user_decision_made.clear()
        self.user_decision_made.wait()

        if self.user_decision in ['abort', 'previous_state', 'teleoperate', 'continue', 'next_state']:
            return self.user_decision
        else:
            return self.query_user()
        
    def query_user_teleop(self):
        self.user_decision = None
        self.user_decision_made.clear()
        self.user_decision_made.wait()

        if self.user_decision == 'continue':
            return 'success'
        elif self.user_decision == 'abort':
            return 'abort'
        else:
            return self.query_user_teleop()


