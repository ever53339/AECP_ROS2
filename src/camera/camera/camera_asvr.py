import argparse
import sys
import rclpy
from rclpy.node import Node
from utils.camera_client import CameraClient
from custom_interfaces.action import DetectSamples
from custom_interfaces.msg import BoundingBox, GantryLocation
from utils.servers import RealServer

import copy
import time

class SampleDetectionActionServer(RealServer):
    """
    camera action server for detect samples.

    Node Name:
        * **sample_detection_action_server** *

    Action Servers:
        * **/detect_samples** (:class:`custom_interfaces.action.DetectSamples`) *

    """
    def __init__(self):
        super().__init__(node_name='sample_detection_action_server',
                         action_name='detect_samples',
                         action_type=DetectSamples)
        # self.publisher_ = self.create_publisher(SampleDetectionRes, 'sample_detection', 10)
        timer_period = 0.1 # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.camera_client = CameraClient('http://localhost:7673')

    # def timer_callback(self):
    #     msg = SampleDetectionRes()
    #     res = self.camera_client.get_sample_detection()
    #     if res['msg'] == 'success':
    #         boxes = []
    #         msg.ids = res['track_ids']

    #         gantry_locs = []
    #         for x, y, w, h in res['boxes']:
    #             boxes.append(BoundingBox(x=x, y=y, w=w, h=h))

    #             x_g, y_g, z_g = self.camera_client.convert_pixel_to_gantry(x, y)
    #             gantry_locs.append(GantryLocation(x_g=x_g, y_g=y_g, z_g=z_g))

    #         msg.boxes = boxes
    #         msg.gantry_locs = gantry_locs
    #         self.publisher_.publish(msg)
    #         self.get_logger().info(f'Publishing sample detection results.')
    #     else:
    #         self.get_logger().error(f'Failed to get sample detection: {res["msg"]}')
    
    def generate_feedback_message(self, elapsed_time):
        """
        Create a feedback message that populates the time elapsed.

        Returns:
            :class:`std_msgs.msg.String`: the populated feedback message
        """
        msg = self.action_type.Feedback() 
        msg.feedback = f'Time elapsed: {elapsed_time:.2} s.'
        return msg

    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        return rclpy.action.server.GoalResponse.ACCEPT
    
    def generate_success_result(self):
        res = self.action_type.Result()
        
        data = copy.deepcopy(self.camera_client.detection_res)

        res.ids = data['ids']
        res.boxes = [BoundingBox(x=box[0], y=box[1], w=box[2], h=box[3]) for box in data['boxes']]
        res.gantry_locs = [GantryLocation(x_g=loc[0], y_g=loc[1], z_g=loc[2]) for loc in data['gantry_locs']]

        return res
        
    async def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Emit 'sample_detection' event to camera socket.io server and wait for the device to be ready.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        self.node.get_logger().info('Executing a goal: dtetect_samples')

        self.camera_client.get_sample_detection()

        freq = 1
        interval = 1.0 / freq
        start_time = time.time()
 
        while True:
            time.sleep(interval)
            elapsed_time = time.time() - start_time
            with self.goal_lock:
                if goal_handle.is_active:
                    if goal_handle.is_cancel_requested:
                        result = self.generate_cancelled_result()
                        message = f'Goal cancelled at {elapsed_time:.2} s'
                        self.node.get_logger().info(message)
                        goal_handle.canceled()
                        return result
                    # ideally would never come to this branch because repeated goals would be rejected
                    elif goal_handle.goal_id != self.goal_handle.goal_id:
                        result = self.generate_preempted_result()
                        message = f'Goal pre-empted at {elapsed_time:.2} s'
                        self.node.get_logger().info(message)
                        goal_handle.abort()
                        return result
                    elif self.camera_client.detection_res['msg'] != '':
                        self.node.get_logger().info(f'Time elapsed: {elapsed_time:.2} s')
                        if self.camera_client.detection_res['msg'] == 'success':
                            result = self.generate_success_result()
                            message = 'Goal executed with success: measure'
                            self.node.get_logger().info(message)
                            goal_handle.succeed()
                        else:
                            result = self.generate_success_result()
                            message = f'Goal execution failed: measure'
                            self.node.get_logger().info(message)
                            goal_handle.abort()
                        return result
                    else:
                        self.node.get_logger().info(f'Time elapsed: {elapsed_time:.2} s')
                        goal_handle.publish_feedback(
                            self.generate_feedback_message(elapsed_time)
                        )
                else:  # ! active
                    self.node.get_logger().info('Goal is no longer active, aborting')
                    result = self.action_type.Result()
                    return result
def main():
    """
    Entry point
    """
    parser = argparse.ArgumentParser(description='Start sample detection action server')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    camera = SampleDetectionActionServer()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(camera.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        camera.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()