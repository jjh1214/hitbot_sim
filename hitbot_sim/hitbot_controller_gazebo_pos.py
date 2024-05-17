import rclpy
import random
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HitbotControllerGazeboPos(Node):
    def __init__(self):
        super().__init__('hitbot_controller_gazebo_pos')
        self.publisher_ = self.create_publisher(JointTrajectory, '/z_arm_controller/joint_trajectory', 10)
        self.manual_mode = self.get_input_mode()

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

    def get_input_mode(self):
        while True:
            mode = input("Enter 'manual' to input positions manually or 'random' for random positions: ")
            if mode.lower() == 'manual':
                return 'manual'
            elif mode.lower() == 'random':
                return 'random'
            else:
                print("Invalid input. Please enter 'manual' or 'random'.")

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = ''
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        if self.manual_mode == 'manual':
            positions = self.get_positions_from_user()
        else:
            positions = self.get_random_positions()
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)

        self.publisher_.publish(msg)

    def get_positions_from_user(self):
        try:
            positions = []
            joint_limits = [(-1.2, 0.0), (-1.571, 1.571), (-2.967, 2.967), (-18.850, 18.850)]
            for i in range(1, 5):
                min_limit, max_limit = joint_limits[i-1]
                pos_str = input(f"Enter position for joint{i} ({min_limit}~{max_limit}): ")
                pos = float(pos_str)
                if pos < min_limit or pos > max_limit:
                    print(f"Position for joint{i} must be between {min_limit} and {max_limit}.")
                    return self.get_positions_from_user()
                positions.append(pos)
            return positions
        except ValueError:
            print("Invalid input. Please enter numerical values.")
            return self.get_positions_from_user()

    def get_random_positions(self):
        positions = []
        joint_limits = [(-1.2, 0.0), (-1.571, 1.571), (-2.967, 2.967), (-18.850, 18.850)]
        for limit in joint_limits:
            min_limit, max_limit = limit
            random_pos = random.uniform(min_limit, max_limit)
            positions.append(random_pos)
        return positions

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = HitbotControllerGazeboPos()
    rclpy.spin(joint_trajectory_publisher)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()