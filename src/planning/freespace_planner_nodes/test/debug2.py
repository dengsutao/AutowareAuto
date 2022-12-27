# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

from ament_index_python import get_package_share_directory

from autoware_auto_planning_msgs.action import PlannerCostmap, PlanTrajectory
from autoware_auto_planning_msgs.srv import ModifyTrajectory

import rclpy
from rclpy.action import ActionServer, ActionClient
import rclpy.node

from launch import LaunchDescription

import launch_ros.actions
import launch_testing

import os
import time
from math import hypot
import unittest


def are_positions_close(p1, p2, margin):
    return hypot(p1.x - p2.x, p1.y - p2.y) < margin


class CostmapGeneratorMockBase(rclpy.node.Node):
    def __init__(self):
        super().__init__('costmap_generator_mock')
        self._action_server = ActionServer(
            self,
            PlannerCostmap,
            '/planning/generate_costmap',
            self.generate_costmap_callback)

    def generate_costmap_callback(self, goal_handle):
        raise NotImplementedError()


class CostmapGeneratorMockSmallSquare(CostmapGeneratorMockBase):
    def __init__(self):
        super().__init__()

    def generate_costmap_callback(self, goal_handle):
        print('in generate_costmap_callback:start')
        goal_handle.succeed()

        result = PlannerCostmap.Result()

        result.costmap.header.stamp = self.get_clock().now().to_msg()
        result.costmap.header.frame_id = "map"

        # 25m x 25m with 0.2m resolution
        result.costmap.info.resolution = 0.2
        result.costmap.info.width = 125
        result.costmap.info.height = 125

        # fully empty space
        result.costmap.data = [0] * (result.costmap.info.width * result.costmap.info.height)

        print('in generate_costmap_callback:returned')
        return result


class CostmapGeneratorMockLongCorridor(CostmapGeneratorMockBase):
    def __init__(self):
        super().__init__()

    def generate_costmap_callback(self, goal_handle):
        goal_handle.succeed()

        result = PlannerCostmap.Result()

        result.costmap.header.stamp = self.get_clock().now().to_msg()
        result.costmap.header.frame_id = "map"

        # 100m x 5m with 0.2m resolution
        result.costmap.info.resolution = 0.2
        result.costmap.info.width = 500
        result.costmap.info.height = 25

        # fully empty space
        result.costmap.data = [0] * (result.costmap.info.width * result.costmap.info.height)

        return result


class PlanTrajectoryClientMock(rclpy.node.Node):
    def __init__(self):
        super().__init__('plan_trajectory_client_mock')
        self._action_client = ActionClient(
            self,
            PlanTrajectory,
            '/planning/plan_parking_trajectory')
        self._send_goal_future = None
        self._get_result_future = None
        self._result = None

    def send_goal(self, goal):
        print('wait for srver:plan_parking_trajectory')
        self._action_client.wait_for_server()
        print('srver:plan_parking_trajectory exists')
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        rclpy.spin_until_future_complete(self, self._get_result_future)

    def get_result_callback(self, future):
        self._result = future.result().result

class ObjectCollisionClientMock(rclpy.node.Node):
    def __init__(self):
        super().__init__('object_collision_estimator_client_mock')
        self.cli = self.create_client(ModifyTrajectory, '/planning/estimate_collision')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ModifyTrajectory.Request()

    def send_request(self, trajectory):
        self.req.original_trajectory = trajectory
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def generate_test_description():
    freespace_planner_node = launch_ros.actions.Node(
        package='freespace_planner_nodes',
        executable='freespace_planner_node_exe',
        parameters=[os.path.join(
            get_package_share_directory('freespace_planner_nodes'),
            'param/test.param.yaml'
        )]
    )

    context = {'freespace_planner_node': freespace_planner_node}

    return LaunchDescription([
        # freespace planner - tested node
        freespace_planner_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


class TestBasicUsage(unittest.TestCase):
    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def send_goal_and_wait_for_result(self, goal, client):
        client.send_goal(goal)
        print('goal has been send.')

        while not client._result:
            print('wait for result')
            time.sleep(0.1)

        print('result has been received')
        return client._result

    def assert_basic_success(self, result):
        # basic success is when return code is SUCCESS and trajectory has at least 2 points
        self.assertTrue(result.result == PlanTrajectory.Result.SUCCESS)
        self.assertTrue(len(result.trajectory.points) > 1)

    def assert_start_and_goal_positions_correct(self, result, goal):
        planned_start = result.trajectory.points[0].pose.position
        requested_start = goal.sub_route.start_pose.position
        START_MARGIN = 0.1  # arbitrary margin
        self.assertTrue(are_positions_close(planned_start, requested_start, START_MARGIN))

        planned_goal = result.trajectory.points[-1].pose.position
        requested_goal = goal.sub_route.goal_pose.position
        GOAL_MARGIN = 2.0  # arbitrary margin based on current params
        self.assertTrue(are_positions_close(planned_goal, requested_goal, GOAL_MARGIN))

    def test_basic_case_works(self, freespace_planner_node):
        # costmap_generator = CostmapGeneratorMockSmallSquare()

        plan_trajectory_client = PlanTrajectoryClientMock()

        goal = PlanTrajectory.Goal()
        goal.sub_route.header.stamp = plan_trajectory_client.get_clock().now().to_msg()
        goal.sub_route.header.frame_id = "odom"
        goal.sub_route.start_pose.position.x = 10.0
        goal.sub_route.start_pose.position.y = 12.5
        goal.sub_route.goal_pose.position.x = 1.0
        goal.sub_route.goal_pose.position.y = 3.5

        result = self.send_goal_and_wait_for_result(goal, plan_trajectory_client)

        self.assert_basic_success(result)

        self.assert_start_and_goal_positions_correct(result, goal)
    
    def test_basic_case_works_object_collision(self, freespace_planner_node):
        costmap_generator = CostmapGeneratorMockSmallSquare()

        plan_trajectory_client = PlanTrajectoryClientMock()

        goal = PlanTrajectory.Goal()
        goal.sub_route.header.stamp = costmap_generator.get_clock().now().to_msg()
        goal.sub_route.header.frame_id = "map"
        goal.sub_route.start_pose.position.x = 1.0
        goal.sub_route.start_pose.position.y = 1.0
        goal.sub_route.goal_pose.position.x = 25.0
        goal.sub_route.goal_pose.position.y = 22.5

        result = self.send_goal_and_wait_for_result(goal,
                                                    plan_trajectory_client,
                                                    costmap_generator)

        plan_trajectory_client = ObjectCollisionClientMock()
        plan_trajectory_client.send_request(result.trajectory)

        print(1)

    def test_long_corridor_trims_points(self, freespace_planner_node):
        costmap_generator = CostmapGeneratorMockLongCorridor()

        plan_trajectory_client = PlanTrajectoryClientMock()

        goal = PlanTrajectory.Goal()
        goal.sub_route.header.stamp = costmap_generator.get_clock().now().to_msg()
        goal.sub_route.header.frame_id = "map"
        goal.sub_route.start_pose.position.x = 4.0
        goal.sub_route.start_pose.position.y = 2.5
        goal.sub_route.goal_pose.position.x = 96.0
        goal.sub_route.goal_pose.position.y = 2.5

        result = self.send_goal_and_wait_for_result(goal,
                                                    plan_trajectory_client,
                                                    costmap_generator)

        self.assert_basic_success(result)

        # assert that trajectory has been trimmed to contain only 100 points
        self.assertTrue(len(result.trajectory.points) == 100)

        self.assert_start_and_goal_positions_correct(result, goal)

if __name__ == '__main__':
    freespace_planner_node = launch_ros.actions.Node(
        package='freespace_planner_nodes',
        executable='freespace_planner_node_exe',
        parameters=[os.path.join(
            get_package_share_directory('freespace_planner_nodes'),
            'param/test.param.yaml'
        )]
    )

    tuc = TestBasicUsage()
    tuc.setUp()
    tuc.test_basic_case_works('')
    # tuc.test_basic_case_works_object_collision('')
    