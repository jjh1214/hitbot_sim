import sys
import os
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from hitbot_msgs.srv import *

os.chdir(os.path.expanduser('~'))
sys.path.append("./hitbot_ws/src/hitbot")  ## get import pass: hitbot_interface.py
from hitbot_interface import HitbotInterface

class HitbotController(Node):
    def __init__(self):
        super().__init__('hitbot_controller')

        self.SetGPIO_srv = self.create_service(SetGPIO, 'set_gpio', self.set_gpio_callback)
        self.GetGPIOOut_srv = self.create_service(GetGPIOOut, 'get_gpio_out', self.get_gpio_out_callback)
        self.GetGPIOIn_srv = self.create_service(GetGPIOIn, 'get_gpio_in', self.get_gpio_in_callback)
        self.SetDragTeach_srv = self.create_service(SetDragTeach, 'set_drag_teach', self.set_drag_teach_callback)
        self.JointHome_srv = self.create_service(JointHome, 'joint_home', self.joint_home_callback)
        self.NewMovejXYZ_srv = self.create_service(NewMovejXYZ, 'new_movej_xyz_lr', self.new_movej_xyz_lr_callback)
        self.NewMovejAngle_srv = self.create_service(NewMovejAngle, 'new_movej_angle', self.new_movej_angle_callback)


        self.hitbot_x = 0
        self.hitbot_y = 0
        self.hitbot_z = 0
        self.hitbot_r = 0

        self.subscription = self.create_subscription(
            Int64,
            '/hitbot_x',
            self.hitbot_x_callback,
            10
        )

        self.subscription = self.create_subscription(
            Int64,
            '/hitbot_y',
            self.hitbot_y_callback,
            10
        )

        self.subscription = self.create_subscription(
            Int64,
            '/hitbot_z',
            self.hitbot_z_callback,
            10
        )

        self.subscription = self.create_subscription(
            Int64,
            '/hitbot_r',
            self.hitbot_r_callback,
            10
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.hitbot_r_publisher = self.create_publisher(Int64, '/hitbot_r', 10)

        self.robot_id = 123  ## 123 is robot_id, Modify it to your own
        self.robot = HitbotInterface(self.robot_id)

        self.init_robot()

    def hitbot_x_callback(self, msg):
        self.hitbot_x = msg.data

    def hitbot_y_callback(self, msg):
        self.hitbot_y = msg.data

    def hitbot_z_callback(self, msg):
        self.hitbot_z = msg.data

    def hitbot_r_callback(self, msg):
        self.hitbot_r = msg.data

    def publish_hitbot_r(self, data):
        msg = Int64()
        msg.data = int(data)
        self.hitbot_r_publisher.publish(msg)

    def joint_states_callback(self, msg):
        try:
            joint_names = msg.name
            joint_positions = msg.position

            if all(joint in joint_names for joint in ['joint1', 'joint2', 'joint3', 'joint4']):
                joint1_index = joint_names.index('joint1')
                joint2_index = joint_names.index('joint2')
                joint3_index = joint_names.index('joint3')
                joint4_index = joint_names.index('joint4')

                joint1 = joint_positions[joint1_index] * 1000
                joint2_angle = joint_positions[joint2_index] * 180 / 3.14
                joint3_angle = joint_positions[joint3_index] * 180 / 3.14
                joint4_angle = joint_positions[joint4_index] * 180 / 3.14

                self.robot.new_movej_angle(joint2_angle, joint3_angle, joint1, joint4_angle, 100, 1)
            else:
                self.get_logger().warning("Required joints not found in /joint_states message")
        except Exception as e:
            self.get_logger().warning(f"Error processing /joint_states message: {str(e)}")

    def set_gpio_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.set_digital_out(request.gpio_number, request.set_on)
                response.success = True
                self.get_logger().info('GPIO setting successful: gpio_number=%d, set_on=%r' % (request.gpio_number, request.set_on))
                break
            except Exception as e:
                self.get_logger().error('Failed to set GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO setting (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to set GPIO.')
                    break

        return response
    
    def get_gpio_out_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.get_digital_out(request.gpio_number)
                response.success = True
                if self.robot.get_digital_out(request.gpio_number) == 1:
                    self.get_logger().info('GPIO : gpio_number=%d is On' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == 0:
                    self.get_logger().info('GPIO : gpio_number=%d is Off' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == -1:
                    self.get_logger().info('GPIO number parameter error')
                elif self.robot.get_digital_out(request.gpio_number) == 3:
                    self.get_logger().info('GPIO Not initialized.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to get GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO status (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to get GPIO.')
                    break

        return response
    
    def get_gpio_in_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.get_digital_out(request.gpio_number)
                response.success = True
                if self.robot.get_digital_out(request.gpio_number) == 1:
                    self.get_logger().info('GPIO : gpio_number=%d is triggered' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == 0:
                    self.get_logger().info('GPIO : gpio_number=%d is not triggered' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == -1:
                    self.get_logger().info('GPIO number parameter error')
                elif self.robot.get_digital_out(request.gpio_number) == 3:
                    self.get_logger().info('GPIO Not initialized.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to get GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO status (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to get GPIO.')
                    break

        return response

    def set_drag_teach_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.set_drag_teach(request.enable)
                response.success = True
                if self.robot.set_drag_teach(request.enable) == True:
                    self.get_logger().info('Successfully set drag teach')
                elif self.robot.set_drag_teach(request.enable) == False:
                    self.get_logger().info('Failed to set drag teach, please check your robot model enable drag teach')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to set drag teach: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying set drag teach (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to set drag teach.')
                    break

        return response

    def joint_home_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.joint_home(request.joint_num)
                response.success = True
                if self.robot.joint_home(request.joint_num) == 0:
                    self.get_logger().info('Joint Not connected. Check your robot')
                elif self.robot.joint_home(request.joint_num) == 1:
                    self.get_logger().info('Success')
                elif self.robot.joint_home(request.joint_num) == 2:
                    self.get_logger().info('Invalid parameters passed')
                elif self.robot.joint_home(request.joint_num) == 3:
                    self.get_logger().info('The robot is still initializing.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to joint home: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying joint home (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to joint home.')
                    break
        return response

    def new_movej_xyz_lr_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.new_movej_xyz_lr(request.goal_x, request.goal_y, request.goal_z, request.goal_r, request.speed, request.roughly, request.lr)
                response.success = True
                if self.robot.new_movej_xyz_lr(request.goal_x, request.goal_y, request.goal_z, request.goal_r, request.speed, request.roughly, request.lr) == 1:
                    self.get_logger().info('Robot Moving')
                else:
                    self.get_logger().info('Error, Check Robot')
                break
            except Exception as e:
                self.get_logger().error('Failed to new_movej_xyz_lr: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying new_movej_xyz_lr (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to new_movej_xyz_lr.')
        return response


    def new_movej_angle_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.new_movej_angle(request.goal_angle1, request.goal_angle2, request.goal_z, request.goal_r, request.speed, request.roughly)
                response.success = True
                if self.robot.new_movej_angle(request.goal_angle1, request.goal_angle2, request.goal_z, request.goal_r, request.speed, request.roughly) == 1:
                    self.get_logger().info('Robot Moving')
                else:
                    self.get_logger().info('Error, Check Robot')
                break
            except Exception as e:
                self.get_logger().error('Failed to new_movej_angle: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying new_movej_angle (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to new_movej_angle.')
        return response

    def init_robot(self):
        self.robot.net_port_initial()
        time.sleep(1)
        is_connected = self.robot.is_connect()

        if is_connected != 1:
            print('No robot connection!!!')
            raise RuntimeError('No robot connection!!!')

        print('Robot connected.')

        init = self.robot.initial(1, 1000) ## 1000 is z-axis parameter, Modify it to your own

        if init != 1:
            print('Robot initialization failed!!!')
            raise RuntimeError('Robot initialization failed!!!')

        print('unlock Robot')
        self.robot.unlock_position()
        print('Robot position initialized.')
        self.robot.new_movej_angle(0, 0, 0, 0, 100, 1)
        self.robot.wait_stop()
        print('Robot I/O output initialized.')
        for i in range(12):
            self.robot.set_digital_out(i, False)
        time.sleep(1)
        print('Robot initialized.')

    def run(self):
        print("hello hibot")

        while rclpy.ok():
            try:
                rclpy.spin_once(self)
            except ValueError as e:
                print("Error:", str(e))
            except RuntimeError as e:
                print("Error:", str(e))

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    hitbot_controller = HitbotController()

    try:
        hitbot_controller.run()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        hitbot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()