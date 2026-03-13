import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from .Inverse_kinematics import inverse
import math

import csv
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class deltanode(Node):

    def __init__(self):
        super().__init__('delta_client')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.logged_positions = []
        self.logged_time = []
        self.start_time = self.get_clock().now()

    ###########################################################################
    ###########################################################################

    def send_goal(self):

        trajectory = JointTrajectory()
        trajectory.joint_names = ['link_0_JOINT_1', 'link_0_JOINT_2', 'link_0_JOINT_3']

        pointi = JointTrajectoryPoint()
        pointf = JointTrajectoryPoint()

        # thetasi = inverse(0.06,0.0,-0.15)
        # thetasf = inverse(-0.09,0.0,-0.18)

        thetasi = (180,180,180)
        thetasf = (300,300,300)

        thetasi = [math.radians(x) for x in thetasi]
        thetasi = [-x for x in thetasi]

        thetasf = [math.radians(x) for x in thetasf]
        thetasf = [-x for x in thetasf]

        if thetasi is None :
            print("IK failed — target outside workspace")

        pointi.positions = thetasi
        pointf.positions = thetasf

        pointi.velocities = [0.0] * 3
        pointf.velocities = [0.0] * 3

        pointi.accelerations = [0.0] * 3
        pointf.accelerations = [0.0] * 3

        pointi.time_from_start = Duration(sec=0,nanosec=200000000)
        pointf.time_from_start = Duration(sec=0,nanosec=600000000)
        # pointi.time_from_start = Duration(sec=1)
        # pointf.time_from_start = Duration(sec=4)

        trajectory.points = [pointi,pointf]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal...')

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)
        return future

    ###########################################################################

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    ###########################################################################

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Trajectory finished!')
        self.save_to_csv()

    ###########################################################################

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback
        positions = list(feedback.actual.positions)

        # ⭐ TIME IN NANOSECONDS (ONLY CHANGE DONE)
        t = (self.get_clock().now() - self.start_time).nanoseconds

        self.logged_positions.append(positions)
        self.logged_time.append(t)

        self.get_logger().info(f"t_ns={t}  q={positions}")

    ###########################################################################

    def save_to_csv(self):

        filename = 'delta_joint_log.csv'

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            writer.writerow([
                'time_nanoseconds',
                'joint_1',
                'joint_2',
                'joint_3'
            ])

            for t, q in zip(self.logged_time, self.logged_positions):
                writer.writerow([t, q[0], q[1], q[2]])

        self.get_logger().info(f"Saved trajectory log to {filename}")

###########################################################################

def main(args=None):
    rclpy.init(args=args)
    delta = deltanode()
    delta.send_goal()
    rclpy.spin(delta)
    rclpy.shutdown()

if __name__ == '__main__':
    main()