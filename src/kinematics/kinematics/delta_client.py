import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from .Inverse_kinematics import inverse
import math

# from s_pathplanner import ptp_scurve
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
        # --- Create trajectory first ---
        trajectory = JointTrajectory()
        trajectory.joint_names = ['link_0_JOINT_1', 'link_0_JOINT_2', 'link_0_JOINT_3']


        # --- Create points ---
        pointi = JointTrajectoryPoint()
        pointf = JointTrajectoryPoint()
        pointh=JointTrajectoryPoint()
        pointh2=JointTrajectoryPoint()
        pointh3=JointTrajectoryPoint()
        pointh4=JointTrajectoryPoint()


        thetasi = inverse(0.06,0.0,-0.15)
        thetasf = inverse(-0.06,0.0,-0.15)
        # home=inverse(0.06,0.0,-0.15)
        # home2=inverse(-0.06,0.0,-0.15)


        thetasi = [math.radians(x) for x in thetasi]
        thetasi = [-x for x in thetasi]

        thetasf = [math.radians(x) for x in thetasf]
        thetasf = [-x for x in thetasf]

        # home = [math.radians(x) for x in home]
        # home = [-x for x in home]

        # home2 = [math.radians(x) for x in home2]
        # home2 = [-x for x in home2]

        # home=[-0.523,0.0,0.0]
        # home2=[0.0,0.0,0.0]
        if thetasi is None :
            print("IK failed — target outside workspace")
    
        pointi.positions = thetasi
        pointf.positions = thetasf
        # pointh.positions =home
        # pointh2.positions =home2

        pointi.velocities = [0.0] * 3
        pointf.velocities = [0.0] * 3
        # pointh.velocities = [0.0] * 3
        # pointh2.velocities = [0.0] * 3

        pointi.accelerations = [0.0] * 3
        pointf.accelerations = [0.0] * 3
        # pointh.accelerations = [0.0] * 3
        # pointh2.accelerations = [0.0] * 3

        pointi.time_from_start = Duration(sec=1)
        pointf.time_from_start = Duration(sec=3)
        # pointh.time_from_start = Duration(sec=2)
        # pointh2.time_from_start = Duration(sec=4)

        trajectory.points = [pointi,pointf]
        # trajectory.points = [pointh]

        # --- Attach trajectory to goal ---
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        # --- Send the goal ---
        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal...')
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return future

    # def send_goal(self):

    # # reset logs for each new goal
    #     self.logged_positions = []
    #     self.logged_time = []
    #     self.start_time = self.get_clock().now()

    #     trajectory = JointTrajectory()
    #     trajectory.joint_names = [
    #         'link_0_JOINT_1',
    #         'link_0_JOINT_2',
    #         'link_0_JOINT_3'
    #     ]

    #     cartesian_targets = []
    #     start = (0.09, 0.0, -0.15)
    #     end   = (-0.09, 0.0, -0.15)

    #     steps = 100
    #     for i in range(steps + 1):
    #         alpha = i / steps
    #         x = start[0] + alpha * (end[0] - start[0])
    #         y = start[1] + alpha * (end[1] - start[1])
    #         z = start[2] + alpha * (end[2] - start[2])
    #         cartesian_targets.append((x, y, z))

    #     time_step = 0.02
    #     current_time = 0.0

    #     for target in cartesian_targets:
    #         thetas = inverse(*target)
    #         if thetas is None:
    #             self.get_logger().error("IK failed — target outside workspace")
    #             return

    #         thetas = [math.radians(x) for x in thetas]
    #         thetas = [-x for x in thetas]

    #         point = JointTrajectoryPoint()
    #         point.positions = thetas
    #         point.velocities = []
    #         point.accelerations = []

    #         current_time += time_step
    #         sec = int(current_time)
    #         nanosec = int((current_time - sec) * 1e9)
    #         point.time_from_start = Duration(sec=sec, nanosec=nanosec)

    #         trajectory.points.append(point)

    #     goal_msg = FollowJointTrajectory.Goal()
    #     goal_msg.trajectory = trajectory

    #     self._action_client.wait_for_server()
    #     self.get_logger().info('Sending multi-point trajectory...')

    #     future = self._action_client.send_goal_async(
    #         goal_msg,
    #         feedback_callback=self.feedback_callback
    #     )
    #     future.add_done_callback(self.goal_response_callback)
    #     return future



    ###########################################
    ###########################################

    def goal_response_callback(self, future):
        goal_handle = future.result()
       
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Trajectory finished!')
        self.save_to_csv()   
        # rclpy.shutdown()
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        positions = list(feedback.actual.positions)

        # time since start (seconds)
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        self.logged_positions.append(positions)
        self.logged_time.append(t)

        self.get_logger().info(f"t={t:.3f}  q={positions}")
        


    def save_to_csv(self):
        filename = 'delta_joint_log.csv'

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Header
            writer.writerow([
                'time',
                'joint_1',
                'joint_2',
                'joint_3'
            ])

            # Data
            for t, q in zip(self.logged_time, self.logged_positions):
                writer.writerow([t, q[0], q[1], q[2]])

        self.get_logger().info(f"Saved trajectory log to {filename}")
    



def main(args=None):
    rclpy.init(args=args)
    delta = deltanode()
    delta.send_goal()
    rclpy.spin(delta)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
