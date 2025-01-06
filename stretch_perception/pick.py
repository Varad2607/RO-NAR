from ultralytics import YOLOWorld
from ultralytics import FastSAM
from ultralytics.models.fastsam import FastSAMPrompt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState

from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np
import cv2
import ikpy.chain
import time

# find the the object center in 3d space and move arm to grasp
class Pick(Node):
    def __init__(self, target_class=3.0):
        super().__init__('pick')
        self.rgb_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, 1)
        self.depth_subscriber = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_cb, 1)
        self.camera_subscriber = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_cb, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(1, self.base_to_camera_tf_cb)
        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_cb, 1)

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            exit(1)

        self.pregrasp()

        self.rgb = None
        self.depth = None
        self.camera = None
        self.base_to_camera_tf = None
        self.joint_state = None

        self.finished = False

        self.processor = self.create_timer(1, self.process_cb)

        self.original_size = None
        self.modified_size = None

        # specify target class here
        self.target_class = target_class

        # put classes here
        self.classes = ['orange cup', 'blue cup', 'red cup', 'green cup']

        # load yolo world
        self.detector = YOLOWorld('yolov8l-world.pt')
        self.detector.set_classes(self.classes)

        # load segmenter

        self.segmenter = FastSAM('FastSAM-x.pt')

    def joint_cb(self, msg):
        self.joint_state = msg

    def depth_cb(self, msg):
        self.depth = msg

    def rgb_cb(self, msg):
        self.rgb = msg

    def camera_cb(self, msg):
        self.camera = msg

    def base_to_camera_tf_cb(self):
        try:
            self.base_to_camera_tf = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            self.get_logger().info(f'Got transform: {self.base_to_camera_tf}')
        except TransformException as ex:
            self.get_logger().error(f'Failed to get transform: {ex}')
            return


    def pregrasp(self):
        grasp_point = JointTrajectoryPoint()
        joint_names = ['joint_wrist_yaw', 'joint_gripper_finger_left']
        positions = [0.01, 0.69]
        grasp_point.positions = positions
        grasp_point.time_from_start = Duration(seconds=2.0).to_msg()

        grasp_goal = FollowJointTrajectory.Goal()
        grasp_goal.trajectory.joint_names = joint_names
        grasp_goal.trajectory.points = [grasp_point]
        self.trajectory_client.send_goal_async(grasp_goal)


    def process_cb(self):
        if self.rgb is None:
            return

        if self.depth is None:
            return
        
        if self.camera is None:
            return
        
        if self.base_to_camera_tf is None:
            return
        
        if self.joint_state is None:
            return

        self.compute()
        self.processor.destroy()


    def compute(self):
        # set camera info
        camera_info = {
            'fx': self.camera.k[0],
            'cx': self.camera.k[2],
            'fy': self.camera.k[4],
            'cy': self.camera.k[5]
        }

        camera_info_rotated = {
            'cx': camera_info['cy'],
            'cy': camera_info['cx'],
            'fx': camera_info['fy'],
            'fy': camera_info['fx']
        }
        
        # preprocess image
        image = cv2.rotate(cv2.cvtColor(np.reshape(np.array(self.rgb.data), (self.rgb.height, self.rgb.width, 3)), 
            cv2.COLOR_RGB2BGR), # color should be in bgr format
            cv2.ROTATE_90_CLOCKWISE) # make sure image is in correct orientation
        
        if self.original_size is None or self.modified_size is None:
            self.original_size = image.shape[:2]
            self.modified_size = (self.original_size[0] - self.original_size[0] % 32, self.original_size[1] - self.original_size[1] % 32)

        # preprocess depth
        depth_data = np.frombuffer(self.depth.data, dtype=np.uint16)
        depth_image = cv2.rotate(np.reshape(depth_data, (self.depth.height, self.depth.width)), cv2.ROTATE_90_CLOCKWISE)

        # detect object
        results = self.detector.predict(image[:self.modified_size[0], :self.modified_size[1], :], imgsz=self.modified_size, conf=0.5)

        frame_classes = results[0].boxes.cls.tolist()
        frame_boxes = results[0].boxes.xyxy.tolist()

        index = frame_classes.index(self.target_class)

        # segment object
        everything_results = self.segmenter(results[0].orig_img, device='cuda', imgsz=self.modified_size)
        prompt_process = FastSAMPrompt(results[0].orig_img, everything_results, device='cuda')
        ann = prompt_process.box_prompt(bbox=frame_boxes[index])
        segment = ann[0].masks.data[0]


        # project segment to 3d object center
        center = (0, 0, 0)
        count = 0
        for i in range(segment.shape[0]):
            for j in range(segment.shape[1]):
                if segment[i][j] > 0:
                    if depth_image[i][j] == 0:
                        continue
                    count += 1

                    # convert to world coordinates
                    x = depth_image[i][j] / 1000
                    y = (camera_info_rotated['cy'] - i) * depth_image[i, j] / camera_info_rotated['fy'] / 1000
                    z = (j - camera_info_rotated['cx']) * depth_image[i, j] / camera_info_rotated['fx'] / 1000

                    center = (center[0] + x, center[1] + y, center[2] + z)

        center = (center[0] / count, center[1] / count, center[2] / count)

        # transform to base_link
        q = self.base_to_camera_tf.transform.rotation
        translation = self.base_to_camera_tf.transform.translation
        transform_matrix = np.array(
            [
                [1-2*q.y**2-2*q.z**2, 2*q.x*q.y-2*q.z*q.w, 2*q.x*q.z+2*q.y*q.w, translation.x],
                [2*q.x*q.y+2*q.z*q.w, 1-2*q.x**2-2*q.z**2, 2*q.y*q.z-2*q.x*q.w, translation.y],
                [2*q.x*q.z-2*q.y*q.w, 2*q.y*q.z+2*q.x*q.w, 1-2*q.x**2-2*q.y**2, translation.z],
                [0, 0, 0, 1]
            ]
        )

        center_wrt_base = np.dot(transform_matrix, np.array([center[0], center[1], center[2], 1]))[:3]

        def get_ik_state(q_lift, q_arml3, q_arml2, q_arml1, q_arml0, q_yaw, q_pitch, q_roll):
            q_base = 0.0
            return [0.0, q_base, 0.0, q_lift, 0.0, q_arml3, q_arml2, q_arml1, q_arml0, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]

        q_init = get_ik_state(
            self.joint_state.position[1],
            self.joint_state.position[2],
            self.joint_state.position[3],
            self.joint_state.position[4],
            self.joint_state.position[5],
            self.joint_state.position[8],
            self.joint_state.position[9],
            self.joint_state.position[10]
        )

        urdf_path = 'stretch.urdf'
        chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
        q_soln = chain.inverse_kinematics(center_wrt_base, initial_position=q_init)

        # add offset
        offset = 0.05

        grasp_pose = {
                "joint_wrist_roll": q_soln[12],
                "joint_wrist_pitch": q_soln[11],
                "joint_wrist_yaw": q_soln[9],
                "translate_mobile_base": q_soln[1],
                "wrist_extension": (q_soln[5] + q_soln[6] + q_soln[7] + q_soln[8] + offset) / 2,
                "joint_lift": q_soln[3],
            }
        
        for name, value in grasp_pose.items():
            self.get_logger().info(f'name, value: {name} {value}')
            point = JointTrajectoryPoint()
            point.positions = [value]
            point.time_from_start = Duration(seconds=5.0).to_msg()

            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [name]
            trajectory_goal.trajectory.points = [point]
            self.trajectory_client.send_goal_async(trajectory_goal)
            time.sleep(3)

        pickup_pose = {
            "wrist_extension": q_soln[5] + q_soln[6] + q_soln[7] + q_soln[8] + offset,
            "joint_gripper_finger_left": -0.1,
        }

        for name, value in pickup_pose.items():
            self.get_logger().info(f'name, value: {name} {value}')
            point = JointTrajectoryPoint()
            point.positions = [value]
            point.time_from_start = Duration(seconds=5.0).to_msg()

            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [name]
            trajectory_goal.trajectory.points = [point]
            self.trajectory_client.send_goal_async(trajectory_goal)
            time.sleep(2)

        lift_pose = {
            "joint_lift": 0.2,
            "wrist_extension": 0.1
        }

        for name, value in lift_pose.items():
            self.get_logger().info(f'name, value: {name} {value}')
            point = JointTrajectoryPoint()
            point.positions = [value]
            point.time_from_start = Duration(seconds=5.0).to_msg()

            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [name]
            trajectory_goal.trajectory.points = [point]
            self.trajectory_client.send_goal_async(trajectory_goal)
            time.sleep(2)

        self.finished = True



def pick(label):
    rclpy.init()

    manipulator = Pick(label)

    while rclpy.ok() and not manipulator.finished:
        rclpy.spin_once(manipulator)

    manipulator.destroy_node()
    rclpy.shutdown()



def main(args=None):
    rclpy.init()

    manipulator = Pick()

    while rclpy.ok() and not manipulator.finished:
        rclpy.spin_once(manipulator)

    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()