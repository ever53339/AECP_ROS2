import rclpy
from rclpy.node import Node
from utils.camera_client import CameraClient
from custom_interfaces.msg import SampleDetectionRes

class SampleDetectionPublisher(Node):

    def __init__(self):
        super().__init__('sample_detection_pub')
        self.publisher_ = self.create_publisher(SampleDetectionRes, 'sample_detection', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.camera_client = CameraClient('http://localhost:7673')
        self.camera_client.calibrate_gantry()

    def timer_callback(self):
        msg = SampleDetectionRes()
        res = self.camera_client.get_sample_detection()
        if res['msg'] == 'success':
            msg.boxes = res['boxes']
            msg.ids = res['track_ids']

            gantry_locs = []
            for x, y, _, _ in res['boxes']:
                x_g, y_g, z_g = self.camera_client.convert_pixel_to_gantry(x, y)
                gantry_locs.append((x_g, y_g, z_g))
            msg.gantry_locs = gantry_locs
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing sample detection results.')
        else:
            self.get_logger().error(f'Failed to get sample detection: {res["msg"]}')

def main(args=None):
    rclpy.init(args=args)

    sample_detection_publisher = SampleDetectionPublisher()

    rclpy.spin(sample_detection_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sample_detection_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
class CameraActionServer(RealServer):
    """
    Camera action server for vision tasks.

    Node Name:
        * **camera_action_server** *

    Action Servers:
        * **/take_measurement** (:class:`custom_interfaces.action.TakeMeasurement`) *

    Publisher: 
        * **/camera_status** (:class:`std_msgs.msg.Int64`) *
    """
    def __init__(self):
        # todo: config action name and type
        super().__init__(node_name='camera_action_server',
                         action_name='take_measurement',
                         action_type=TakeMeasurement)

        self.camera_client = CameraClient('http://localhost:7673')

        # In addition to the action server, create a publisher publishing the device status continuously
        self._publisher = self.node.create_publisher(msg_type=String, topic='sample_detection', qos_profile=10)
        
        self.timer = self.node.create_timer(timer_period_sec=0.5, callback=self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.z300_ctrl.dev_status
        self._publisher.publish(msg)
    
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
        if self.z300_ctrl.is_device_ready():
            self.node.get_logger().info('Received and accepted a goal: measurement')
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.node.get_logger().info('Received and rejected a goal because z300 is already running')
            return rclpy.action.server.GoalResponse.REJECT
        
    async def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Emit 'measure' event to LIBS socket.io server and wait for the device to be ready.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        self.node.get_logger().info('Executing a goal: measurement')
        
        self.z300_ctrl.measure()

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
                    elif self.z300_ctrl.res['measure']['msg'] is not None:
                        self.node.get_logger().info(f'Time elapsed: {elapsed_time:.2} s')
                        if self.z300_ctrl.res['measure']['msg'] == 'success':
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
    parser = argparse.ArgumentParser(description='Start z300 measure action server')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)
    rclpy.init()  # picks up sys.argv automagically internally
    z300 = Z300MeasureActionServer()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(z300.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        z300.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
'''