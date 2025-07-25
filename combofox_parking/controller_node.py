import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from std_srvs.srv import Trigger

from ibt_ros2_interfaces.action import MoveArm
from ibt_ros2_interfaces.msg import MoveReq, Waypoint
from combo_box_interfaces.action import DetectBoxes

# ANSI color codes for logging
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[94m'
YELLOW = '\033[93m'
RESET = '\033[0m'

class ComboFoxController(Node):
    """
    This class implements the main controller node for the combo box picking task.
    It orchestrates robot movement, vision-based box detection, and gripper control
    to perform a pick-and-place pipeline.
    """
    def __init__(self):
        """
        Initializes the ComboBoxController node, creating clients for various services and actions,
        defining robot poses, and setting up TF2 listeners and broadcasters.
        """
        super().__init__('combofox_controller')
        self.get_logger().info(f'{YELLOW}Starting ComboFoxController...{RESET}')

        # Create clients for robot services and actions
        self.rearm_client = self.create_client(Trigger, '/robofox/rearm')
        self.disarm_client = self.create_client(Trigger, '/robofox/disarm')
        self.move_client = ActionClient(self, MoveArm, '/robofox/move_arm')

        # Timeout for service and action calls
        self.timeout_sec = 100.0

        self.last_feedback_status = None

        # PARKING POSE DEFINITION
        self.paking_pose_joints = [0.0000, 0.9948, 2.2515, -0.5864, 1.0629, 0.0000]

    def call_rearm_service(self):
        """
        Calls the /robofox/rearm service to enable the robot arm.
        
        :return: True if the service call was successful, False otherwise.
        :rtype: bool
        """
        if not self.rearm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'{RED}Rearm service not available.{RESET}')
            return False

        request = Trigger.Request()
        future = self.rearm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        res = future.result()

        if res is not None and res.success:
            self.get_logger().info(f'{GREEN}Rearm service called successfully.{RESET}')
            # Timeout of 5 seconds after rearm to ensure the arm is ready
            time.sleep(5.0)
            return True
        else:
            self.get_logger().error(f'{RED}Failed to call rearm service.{RESET}')
            return False
    
    def call_disarm_service(self):
        """
        Calls the /robofox/disarm service to disable the robot arm.
        
        :return: True if the service call was successful, False otherwise.
        :rtype: bool
        """
        if not self.disarm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'{RED}Disarm service not available.{RESET}')
            return False

        request = Trigger.Request()
        future = self.disarm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        res = future.result()

        if res is not None and res.success:
            self.get_logger().info(f'{GREEN}Disarm service called successfully.{RESET}')
            # Timeout of 1 second after disarm to ensure the arm is not engaged
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f'{RED}Failed to call disarm service.{RESET}')
            return False
    
    def feedback_callback(self, feedback_msg):
        """
        Callback function to log feedback status from an action server.
        
        :param feedback_msg: The feedback message from the action server.
        """
        current_status = feedback_msg.feedback.status
        if current_status != self.last_feedback_status:
            self.last_feedback_status = current_status
            self.get_logger().info(f'{YELLOW} Feedback: {current_status}{RESET}')
            
    def move_to_req_pose(self, req: MoveReq):
        """
        Sends a single MoveReq goal to the MoveArm action server and waits for the result.
        
        :param req: The movement request message.
        :type req: MoveReq
        :return: True if the movement was successful, False otherwise.
        :rtype: bool
        """
        self.get_logger().info(f'{BLUE} [Trying to move to requested pose...{RESET}')
        goal_msg = MoveArm.Goal()
        goal_msg.requests = [req]

        self.get_logger().info(f'{BLUE}Sending goal: {req.move_type}{RESET}')
        future = self.move_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'{RED}Goal was rejected or failed to send.{RESET}')
            return False

        self.get_logger().info(f'{BLUE}Goal accepted, waiting for result...{RESET}')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
        result = result_future.result().result

        self.get_logger().info(f'{YELLOW}Move result: {result.error_str} (Code: {result.error_code}){RESET}')
        return True
    
    def move_to_joint_pose(self, joint_pose):
        """
        Moves the robot to a specified joint configuration.
        
        :param joint_pose: A list of target joint angles in radians.
        :type joint_pose: list[float]
        :return: True if the movement was successful, False otherwise.
        :rtype: bool
        """
        wp = Waypoint()
        wp.pose = joint_pose
        wp.smoothing_factor = 0.1
        wp.next_segment_velocity_factor = 0.5

        req = MoveReq()
        req.move_type = MoveReq.JOINT
        req.velocity = 0.05
        req.acceleration = 0.1
        req.rotational_velocity = 0.05
        req.rotational_acceleration = 0.1
        req.waypoints = [wp]

        return self.move_to_req_pose(req)

        
    def execute_pipeline(self):
        """
        Executes the main operational pipeline:
        1. Rearm robot and wait for servers.
        2. Move to parking pose.
        3. Disarm the robot upon completion or failure.
        """
        if not self.call_rearm_service():
            self.get_logger().error(f'{RED}Rearm service call failed. Exiting...{RESET}')
            return

        self.get_logger().info(f'{YELLOW} Waiting for action servers...{RESET}')
        self.move_client.wait_for_server()
        self.get_logger().info(f'{GREEN} Move arm server active...{RESET}')

        if not self.move_to_joint_pose(self.paking_pose_joints):
            self.get_logger().error(f'{RED}Pipeline aborted: failed to reach parking pose.{RESET}')
            return
            
        if not self.call_disarm_service():
            self.get_logger().error(f'{RED}Disarm service call failed. Exiting...{RESET}')
            return

        self.get_logger().error(f'{RED}Detection failed after all poses.{RESET}')
        return

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = ComboFoxController()
    try:
        node.execute_pipeline()
    except KeyboardInterrupt:
        node.get_logger().info(f'{RED}ComboFoxController interrupted by user.{RESET}')
    finally:
        # Distrugge solo il nodo, senza spegnere tutto il sistema
        node.get_logger().info('Parking sequence finished. Shutting down node.')
        node.destroy_node()
        # rclpy.shutdown() 

if __name__ == '__main__':
    main()
