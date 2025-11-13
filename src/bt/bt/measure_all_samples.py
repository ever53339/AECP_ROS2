from ast import main
import py_trees
import py_trees_ros
import rclpy
import typing
import sys
import py_trees.console as console
import py_trees_ros.subscribers as subscribers
import action_msgs.msg as action_msgs

from rclpy.node import Node
from std_msgs.msg import String
from py_trees.behaviours import Failure
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from py_trees import logging
from py_trees.composites import Sequence, Selector, Parallel
from custom_interfaces.action import DetectSamples, TakeMeasurement, MoveGantry, ExportSpectrum, AnalyzeSpectrum
from custom_interfaces.msg import SampleDetectionRes
from rcl_interfaces.msg import ParameterDescriptor
from py_trees.idioms import pick_up_where_you_left_off

def get_gcode_from_pixel_coordinates(x: float, y: float) -> str:
    """Generate a gcode script that visits a single position.
    All units are millimeter.
    """
    return f'G17 G21 G90 \n G00 X{x*1000:.0f} Y{y*1000:.0f} \n'


# class DetectSampleActionClient(py_trees_ros.action_clients.FromConstant):
#     def __init__(self, name: str):
#         super().__init__(name=name,
#                          action_type=DetectSamples,
#                          action_name='detect_samples',
#                          action_goal=DetectSamples.Goal(),
#                          )
        
#         self.bb = self.attach_blackboard_client(name=name)

#         self.bb.register_key('ids', access=Access.WRITE)
#         self.bb.register_key('gantry_locs', access=Access.WRITE)
    
#     def update(self):
#         """
#         Check only to see whether the underlying action server has
#         succeeded, is running, or has cancelled/aborted for some reason and
#         map these to the usual behaviour return states.

#         Returns:
#             :class:`py_trees.common.Status`
#         """
#         self.logger.debug("{}.update()".format(self.qualified_name))

#         if self.send_goal_future is None:
#             self.feedback_message = "no goal to send"
#             return py_trees.common.Status.FAILURE
#         if self.goal_handle is not None and not self.goal_handle.accepted:
#             # goal was rejected
#             self.feedback_message = "goal rejected"
#             return py_trees.common.Status.FAILURE
#         if self.result_status is None:
#             return py_trees.common.Status.RUNNING
#         elif not self.get_result_future.done():
#             # should never get here
#             self.node.get_logger().warn("got result, but future not yet done [{}]".format(self.qualified_name))
#             return py_trees.common.Status.RUNNING
#         else:
#             self.node.get_logger().debug("goal result [{}]".format(self.qualified_name))
#             self.node.get_logger().debug("  status: {}".format(self.result_status_string))
#             self.node.get_logger().debug("  message: {}".format(self.result_message))
#             if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:  # noqa
#                 self.feedback_message = "successfully completed"
#                 return py_trees.common.Status.SUCCESS
#             else:
#                 self.feedback_message = "failed"
#                 return py_trees.common.Status.FAILURE

class AreAllSampleMeasured(Behaviour):
    def __init__(self, name: str, detect_behavior):
        super().__init__(name)

        self.detect_behavior = detect_behavior

        self.bb = self.attach_blackboard_client(name="are all sample measured?")
        # self.bb.register_key('ids', access=Access.READ)
        # self.bb.register_key('gantry_locs', access=Access.READ)
        self.bb.register_key('measured_ids', access=Access.READ)
        self.bb.register_key('next_sample_loc', access=Access.WRITE)
        self.bb.register_key('next_sample_id', access=Access.WRITE)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')


        # Check if the action behaviour succeeded in the previous tick
        if self.detect_behavior.status == py_trees.common.Status.SUCCESS:
            # Access the result data directly from the action behaviour instance
            self.result_value = self.detect_behavior.result_message.result
            # self.feedback_message = f"Action succeeded, result: {self.result_value}."
            ids, gantry_locs = self.result_value.ids, self.result_value.gantry_locs

            for ids, gantry_loc in zip(ids, gantry_locs):
                if ids not in self.bb.measured_ids:
                    self.bb.set(name='next_sample_id', value=ids)
                    self.bb.set(name='next_sample_loc', value=gantry_loc)
                    return py_trees.common.Status.FAILURE
            
            self.bb.set(name='next_sample_id', value=None)
            self.bb.set(name='next_sample_loc', value=None)
            return py_trees.common.Status.SUCCESS
        
        elif self.detect_behavior.status == py_trees.common.Status.FAILURE:
            self.feedback_message = "Action failed"
            return py_trees.common.Status.FAILURE
        else:
            # Still running or not started yet
            return py_trees.common.Status.RUNNING
        
        '--------------------------------------------------'
        # for i, id in enumerate(self.bb.ids):
        #     if id not in self.bb.measured_ids:
        #         # self.bb.measured_ids.add(id)
        #         self.bb.set(name='next_sample_index', value=i)
        #         self.bb.set(name='next_sample_id', value=id)
        #         return Status.FAILURE
        
        # self.bb.set(name='next_sample_index', value=None)
        # self.bb.set(name='next_sample_id', value=None)
        # return Status.SUCCESS

class SetNextSample(Behaviour):
    def __init__(self, name: str, detect_behavior):
        super().__init__(name)  
        
        self.detect_behavior = detect_behavior

        self.bb = self.attach_blackboard_client(name="set next sample")
        # self.bb.register_key('gcode', access=Access.WRITE)
        self.bb.register_key('gantry_command', access=Access.WRITE)
        self.bb.register_key('gantry_command_z', access=Access.WRITE)
        # self.bb.register_key('gantry_locs', access=Access.READ)
        self.bb.register_key('next_sample_loc', access=Access.READ)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')

        if self.bb.next_sample_loc is None:
            self.logger.debug(f'No samples need to be measured.')
            return Status.FAILURE
        
        x_g, y_g, z_g = self.bb.next_sample_loc.x_g, self.bb.next_sample_loc.y_g, self.bb.next_sample_loc.z_g
        
        goal = MoveGantry.Goal()
        goal.cmd = get_gcode_from_pixel_coordinates(x_g, y_g)
        self.bb.set(name='gantry_command', value=goal)

        goal_z = MoveGantry.Goal()
        goal_z.cmd = f'G17 G21 G90 \n G00 Z{z_g*1000:.0f} \n'
        self.bb.set(name='gantry_command_z', value=goal_z)

        return Status.SUCCESS

class UpdateLeftSamples(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.bb = self.attach_blackboard_client(name="update left samles")
        self.bb.register_key('next_sample_id', access=Access.WRITE)
        self.bb.register_key('measured_ids', access=Access.WRITE)
        # self.bb.register_key('gantry_command', access=Access.WRITE)

    def setup(self, **kwargs: typing.Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        if self.bb.next_sample_id is None:
            self.logger.debug(f'No samples need to be measured.')
            return Status.FAILURE
        
        # goal = MoveGantry.Goal()
        # goal.cmd = self.bb.gcode.pop()
        # self.bb.gcode.pop()
        self.bb.measured_ids.add(self.bb.next_sample_id)
        # self.bb.set(name='gantry_command', value=goal)

        return Status.SUCCESS
'''
def gen_gcode(x_interval: float, y_interval: float, x_points: int, y_points: int, z_safe: float) -> list[str]:
    """Generate a gcode script that visits positions on a grid.
    All units are millimeter.

    Args:
        x_interval (float): inverval between two adjacent positions on x-scale.
        y_interval (float): inverval between two adjacent positions on x-scale.
        x_points (int): number of positions on x-scale.
        y_points (int): number of positions on y-scale.
        z_safe (float): safety distance on z-scale used when traveling between positions.
    Returns:
        list[str]: generated gcode script.
    """
    gcode = []


    x_pos, y_pos = [], []
 
    for i in range(x_points):
        x_pos.append(x_interval * i)

    for i in range(y_points):
        y_pos.append(y_interval * i)
    
    for x in x_pos:
        for y in y_pos:
            gcode.append('G17 G21 G90 ' + '\n'
                         f'G00 X{x} Y{y}' + '\n')
    
    return gcode[::-1]
'''

def create_root():
    root = Parallel(
        name="mini_platform",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics_to_bb = Sequence(name="topics_to_bb", memory=True)

    gantry_to_bb = subscribers.ToBlackboard(name='gantry_to_bb', 
                                            topic_name='gantry_status',
                                            topic_type=String,
                                            blackboard_variables={'gantry_status': 'data'},
                                            qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                            )
    
    analytical_to_bb = subscribers.ToBlackboard(name='anal_dev_to_bb', 
                                                topic_name='z300_status',
                                                topic_type=String,
                                                blackboard_variables={'z300_status': 'data'},
                                                qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                                )

    # camera_to_bb = subscribers.ToBlackboard(name='camera_to_bb', 
    #                                             topic_name='sample_detection',
    #                                             topic_type=SampleDetectionRes,
    #                                             blackboard_variables={'boxes': 'boxes', 'ids': 'ids', 'gantry_locs': 'gantry_locs'},
    #                                             qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
    #                                             )

    main_branch = Sequence(name='main branch', memory=True)

    tasks = Selector(name='main task', memory=False)
    
    move_aside = py_trees_ros.action_clients.FromConstant(name='move_aside',
                                                      action_type=MoveGantry,
                                                      action_name='move_gantry',
                                                      action_goal=MoveGantry.Goal(cmd='G17 G21 G90 \n G00 Z40 \n G00 X0 Y200 \n'), # todo: set outside location
                                                      )

    detect_samples = py_trees_ros.action_clients.FromConstant(name='detect_samples',
                                                      action_type=DetectSamples,
                                                      action_name='detect_samples',
                                                      action_goal=DetectSamples.Goal(),
                                                      )

    all_samples_measured = AreAllSampleMeasured('are all samples measured?', detect_samples)

    safety = Failure(name='not safe?')

    # measure_one_sample = Sequence('measure one sample', True)
    
    set_next_goal = SetNextSample('set next goal', detect_samples)

    wait_for_goal = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForGoal",
        variable_name="/gantry_command"
    )

    # wait_for_goal_z = py_trees.behaviours.WaitForBlackboardVariable(
    #     name="WaitForGoal",
    #     variable_name="/gantry_command_z"
    # )

    

    move = Sequence(name='move', memory=True)

    move_up = py_trees_ros.action_clients.FromConstant(name='move_up',
                                                      action_type=MoveGantry,
                                                      action_name='move_gantry',
                                                      action_goal=MoveGantry.Goal(cmd='G17 G21 G90 \n G00 Z40 \n'),
                                                      )
    
    move_xy = py_trees_ros.action_clients.FromBlackboard(name='move_xy',
                                                      action_type=MoveGantry,
                                                      action_name='move_gantry',
                                                      key='gantry_command',
                                                      )
    move_down = py_trees_ros.action_clients.FromBlackboard(name='move_down',
                                                      action_type=MoveGantry,
                                                      action_name='move_gantry',
                                                      key='gantry_command_z',
                                                      )
    
    # move_down = py_trees_ros.action_clients.FromConstant(name='move_down',
    #                                                   action_type=MoveGantry,
    #                                                   action_name='move_gantry',
    #                                                   action_goal=MoveGantry.Goal(cmd='G17 G21 G90 \n G00 Z0 \n'),
    #                                                   )
    
    scan_one_point = Sequence(name='scan one point', memory=True)

    measure = py_trees_ros.action_clients.FromConstant(name='measure',
                                                       action_type=TakeMeasurement,
                                                       action_name='take_measurement',
                                                       action_goal=TakeMeasurement.Goal(),
                                                       )
    
    export = py_trees_ros.action_clients.FromConstant(name='export',
                                                      action_type=ExportSpectrum,
                                                      action_name='export_spectrum',
                                                      action_goal=ExportSpectrum.Goal())
    
    analyze = py_trees_ros.action_clients.FromConstant(name='analyze',
                                                      action_type=AnalyzeSpectrum,
                                                      action_name='analyze_spectrum',
                                                      action_goal=AnalyzeSpectrum.Goal())
    
    # update_left_samples = UpdateLeftSamples('update left samples')

    root.add_children([topics_to_bb, main_branch])

    main_branch.add_children([move_aside, detect_samples, tasks])

    topics_to_bb.add_children([gantry_to_bb, analytical_to_bb])


    # measure_one_sample.add_children([set_next_goal, wait_for_goal, move, scan_one_point, update_left_samples])

    move.add_children([move_up, move_xy, move_down])

    scan_one_point.add_children([measure, export, analyze])

    measure_one_sample = pick_up_where_you_left_off(name='measure one sample', tasks=[set_next_goal, wait_for_goal, move, scan_one_point])

    tasks.add_children([safety, all_samples_measured, measure_one_sample])

    return root

def main():
    logging.level = logging.Level.DEBUG

    rclpy.init(args=None)
    # create a ROS node and declare parameters
    node = Node('behavior_tree')
    # x_gap, y_gap, x_points, y_points = node.declare_parameters(
    #     namespace='',
    #     parameters=[('x_gap', 0.2, ParameterDescriptor(description='The gap in mm between two points in X direction.')),
    #                 ('y_gap', 0.2, ParameterDescriptor(description='The gap in mm between two points in Y direction.')),
    #                 ('x_points', 51, ParameterDescriptor(description='The number points to be scanned in X direction.')),
    #                 ('y_points', 51, ParameterDescriptor(description='The number of points to be scanned in Y direction.'))
    #                 ])

    # node.get_logger().info(f'x gap: {x_gap.value}, y gap: {y_gap.value}, x points: {x_points.value}, y points: {y_points.value}')
    blackboard = py_trees.blackboard.Client(name='Global')
    blackboard.register_key('gantry_status', access=Access.READ)
    blackboard.register_key('z300_status', access=Access.READ)

    # blackboard.register_key('boxes', access=Access.READ)
    # blackboard.register_key('ids', access=Access.WRITE)
    # blackboard.register_key('gantry_locs', access=Access.WRITE)
    blackboard.register_key('measured_ids', access=Access.WRITE)
    # blackboard.register_key('gcode', access=Access.WRITE)
    blackboard.register_key('next_sample_loc', access=Access.WRITE)
    blackboard.register_key('next_sample_id', access=Access.WRITE)
    blackboard.register_key('gantry_command', access=Access.WRITE)
    blackboard.register_key('gantry_command_z', access=Access.WRITE)
    
    # blackboard.ids, blackboard.gantry_locs = [], []
    blackboard.measured_ids = set()
    # blackboard.gcode = gen_gcode(x_gap.value, y_gap.value, x_points.value, y_points.value, 10)
    blackboard.next_sample_loc = None
    blackboard.next_sample_id = None
    root = create_root()

    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node=node, timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(500)


    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
