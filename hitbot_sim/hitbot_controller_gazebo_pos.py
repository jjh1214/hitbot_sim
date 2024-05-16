import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/z_arm_controller/joint_trajectory', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = ''
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        positions = self.get_positions_from_user()
        
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

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()