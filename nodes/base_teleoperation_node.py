"""
Only publishes.
"""

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import math
from math import sin, cos, tan, pi, fabs, radians
import sys

# third-party library imports
import rospy

# import ros messages
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

# import custom modules (Autoware Control and NewEagle to control both simultaneously).
from autoware_msgs.msg import AccelCmd, BrakeCmd, ControlCommand, SteerCmd, VehicleCmd
from raptor_dbw_msgs.msg import AcceleratorPedalCmd, SteeringCmd, GearCmd, MiscCmd, GlobalEnableCmd
from raptor_dbw_msgs.msg import ActuatorControlMode, Gear, TurnSignal
from raptor_dbw_msgs.msg import BrakeCmd as BrakePedalCmd
from BaseTeleoperation import BaseTeleoperation

__author__ = 'Boluwatife Olabiran'
__license__ = 'GPLv3'
__maintainer__ = 'Boluwatife Olabiran'
__email__ = 'bso19a@fsu.edu'


class BaseTeleoperationNode(BaseTeleoperation):
    """docstring for ClassName"""

    def __init__(self, max_speed, max_steering_angle, max_steering_wheel_angle,
                 max_acceleration, max_acceleration_actuation, max_brake_actuation, speed_increment,
                 pub_mode, pub_dbw_message_stamped, stamped,
                 ignore, enable, count, svel,
                 global_enable_status, global_disable_status, clip_steering_angle, control_type,
                 coast_mode, mirror_speed_with_gear_command, ):
        """Constructor for BaseTeleoperationNode"""
        # get parameters needed by BaseTeleoperation here

        # override the instance attributes
        super(BaseTeleoperationNode, self).__init__(enable=True)

        # actions

        # temporary variables (to fix triggers not starting properly)
        self.RT_press_count = False
        self.LT_press_count = False

        # setup threads to run asynchronously
        period = 0.02  # a.k.a. duration in s (or 1/Hz)

        # instantiate publishers (todo: set queue size and topic names as inputs to the attribute)
        publisher_queue_size = 1
        # global enable and disable command status
        self.global_enable_cmd_pub = rospy.Publisher("enable_cmd", Bool, queue_size=publisher_queue_size)
        rospy.Timer(rospy.Duration(period), self.enable_cmd_publisher,
                    oneshot=False, reset=True)
        self.global_disable_cmd_pub = rospy.Publisher("disable_cmd", Bool, queue_size=publisher_queue_size)
        rospy.Timer(rospy.Duration(period), self.disable_cmd_publisher,
                    oneshot=False, reset=True)

        # choose actuator ROS message type and topic
        '''
        ToDo: create a custom message to publish vehicle commands. Create new nodes to translate my message
        to the below messages.        
        '''
        if self.pub_mode in ('vehicle_cmd', 'all'):
            self.vehicle_cmd_pub = rospy.Publisher("vehicle_cmd", VehicleCmd, queue_size=publisher_queue_size)
            rospy.Timer(rospy.Duration(self.loop_sample_time), self.autoware_control_command_publisher,
                        oneshot=False, reset=True)

        if self.pub_mode in ('twist', 'all'):
            self.twist_pub = rospy.Publisher("cmd_vel", TwistStamped, queue_size=publisher_queue_size) \
                if self.stamped \
                else rospy.Publisher("cmd_vel", Twist, queue_size=publisher_queue_size)
            rospy.Timer(rospy.Duration(self.loop_sample_time), self.twist_publisher,
                        oneshot=False, reset=True)

        # default
        if self.pub_mode in ('ackermann', 'all') or self.pub_mode not in ('vehicle_cmd', 'twist'):
            # default to AckermannDrive or AckermannDriveStamped
            self.ackermann_drive_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped,
                                                       queue_size=publisher_queue_size) \
                if self.stamped \
                else rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=publisher_queue_size)
            rospy.Timer(rospy.Duration(self.loop_sample_time), self.ackermann_drive_publisher,
                        oneshot=False, reset=True)

        if self.pub_dbw_message:
            '''
            ToDo: remove this node and set up another node to do the publishing to New Eagles raptor.
            '''
            # # get NewEagle raptor_dbw parameters
            # self.ignore = rospy.get_param('~ignore', False)  # Ignore driver overrides
            # self.enable = rospy.get_param('~enable', True)  # Use enable and disable buttons
            # self.count = rospy.get_param('~count', False)  # Increment counter to enable watchdog
            # self.svel = rospy.get_param('~svel', 0.0)  # Steering command speed

            self.pub_accelerator_pedal = rospy.Publisher("accelerator_pedal_cmd", AcceleratorPedalCmd,
                                                         queue_size=publisher_queue_size)
            self.pub_brake = rospy.Publisher("brake_cmd", BrakePedalCmd, queue_size=publisher_queue_size)
            self.pub_misc = rospy.Publisher("misc_cmd", MiscCmd, queue_size=publisher_queue_size)
            self.pub_steering = rospy.Publisher("steering_cmd", SteeringCmd, queue_size=publisher_queue_size)
            self.pub_global_enable = rospy.Publisher("global_enable_cmd", GlobalEnableCmd,
                                                     queue_size=publisher_queue_size)
            self.pub_gear = rospy.Publisher("gear_cmd", GearCmd, queue_size=publisher_queue_size)

            rospy.Timer(rospy.Duration(self.loop_sample_time), self.new_eagle_dbw_publisher,
                        oneshot=False, reset=True)

        if self.enable:
            self.pub_enable = rospy.Publisher("enable", Empty, queue_size=publisher_queue_size)
            self.pub_disable = rospy.Publisher("disable", Empty, queue_size=publisher_queue_size)

        self.autonomy_mode_cmd_pub = rospy.Publisher("autonomy_mode", Bool, queue_size=publisher_queue_size)
        rospy.Timer(rospy.Duration(period), self.autonomy_mode_cmd_publisher,
                    oneshot=False, reset=True)

    def ackermann_drive_publisher(self, event):
        # Detect joy timeouts and reset
        self.detect_and_reset_joy_timeout(event)

        # AckermannDrive message
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.steering_angle = float(self.joystick_data['steering_joy'])  # desired virtual angle (radians)
        # ackermann_cmd_msg.steering_angle_velocity = float(0.0)  # desired rate of change (radians/s)
        ackermann_cmd_msg.speed = float(self.joystick_data['speed_joy'])  # desired forward speed (m/s)
        # ackermann_cmd_msg.acceleration = float(0.0)  # desired acceleration (m/s^2)
        # ackermann_cmd_msg.jerk = float(0.0)  # desired jerk (m/s^3)

        if self.stamped:
            # AckermannDriveStamped message
            ackermann_stamped_cmd_msg = AckermannDriveStamped()
            ackermann_stamped_cmd_msg.header.stamp = rospy.Time.now()
            ackermann_stamped_cmd_msg.header.frame_id = 'joy_ackermann'
            ackermann_stamped_cmd_msg.drive = ackermann_cmd_msg
            self.ackermann_drive_pub.publish(ackermann_stamped_cmd_msg)
            return
        self.ackermann_drive_pub.publish(ackermann_cmd_msg)

    def autoware_control_command_publisher(self, event):
        # Detect joy timeouts and reset
        self.detect_and_reset_joy_timeout(event)

        # publish to /accel_cmd
        accel_cmd = AccelCmd()
        accel_cmd.header.stamp = rospy.Time.now()
        accel_cmd.header.frame_id = "acceleration"
        accel_cmd.accel = int(self.joystick_data["accelerator_pedal_joy"] * 100)
        # self.accel_pub.publish(accel_cmd)

        # publish to /brake_cmd
        brake_cmd = BrakeCmd()
        brake_cmd.header.stamp = rospy.Time.now()
        brake_cmd.header.frame_id = "brake"
        brake_cmd.brake = int(abs(self.joystick_data["brake_joy"]) * 100)
        # self.brake_pub.publish(brake_cmd)

        # publish to /steer_cmd
        steer_cmd = SteerCmd()
        steer_cmd.header.stamp = rospy.Time.now()
        steer_cmd.header.frame_id = "steering"
        steer_cmd.steer = int(self.joystick_data["steering_wheel_joy"])
        # self.steer_pub.publish(steer_cmd)

        # publish to /twist_cmd
        speed = self.joystick_data['speed_joy']
        steering = self.joystick_data['steering_joy']
        yaw_rate = self.convert_ackermann_steering_angle_to_yaw_rate(speed, steering)
        twist_cmd = TwistStamped()
        twist_cmd.header.stamp = rospy.Time.now()
        twist_cmd.header.frame_id = "twist"
        twist_cmd.twist.linear.x = float(speed)  # desired forward speed (m/s)
        # twist_cmd.twist.linear.y = 0.0
        # twist_cmd.twist.linear.z = 0.0
        # twist_cmd.twist.angular.x = 0.0
        # twist_cmd.twist.angular.y = 0.0
        twist_cmd.twist.angular.z = float(yaw_rate)  # desired yaw_rate (rads/s)
        # self.twist_pub.publish(twist_cmd)

        # publish to ctrl_cmd
        ctrl_cmd_msg = ControlCommand()
        ctrl_cmd_msg.linear_velocity = float(self.joystick_data['speed_joy'])  # desired forward speed (m/s)
        # ctrl_cmd_msg.linear_acceleration = float(0.0)  # desired acceleration (m/s^2)
        ctrl_cmd_msg.steering_angle = float(self.joystick_data['steering_joy'])  # desired virtual angle (radians)

        # publish to /vehicle_cmd
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = rospy.Time.now()
        vehicle_cmd.header.frame_id = "vehicle"
        # vehicle_cmd.steer_cmd = steer_cmd  # steer_cmd
        # vehicle_cmd.accel_cmd = accel_cmd  # accel_cmd
        # vehicle_cmd.brake_cmd = brake_cmd  # brake_cmd
        # vehicle_cmd.twist_cmd = twist_cmd  # twist_cmd
        vehicle_cmd.ctrl_cmd = ctrl_cmd_msg
        self.vehicle_cmd_pub.publish(vehicle_cmd)

    def twist_publisher(self, event):
        # Detect joy timeouts and reset
        self.detect_and_reset_joy_timeout(event)
        twist_cmd_msg = Twist()
        speed = self.joystick_data['speed_joy']
        steering = self.joystick_data['steering_joy']
        yaw_rate = self.convert_ackermann_steering_angle_to_yaw_rate(speed, steering)
        twist_cmd_msg.linear.x = float(speed)  # desired forward speed (m/s)
        twist_cmd_msg.angular.z = float(yaw_rate)  # desired yaw_rate (rads/s)

        if self.stamped:
            twist_stamped_cmd_msg = TwistStamped()
            twist_stamped_cmd_msg.header.stamp = rospy.Time.now()
            twist_stamped_cmd_msg.header.frame_id = 'joy_twist'
            twist_stamped_cmd_msg.twist = twist_cmd_msg
            self.twist_pub.publish(twist_stamped_cmd_msg)
            return
        self.twist_pub.publish(twist_cmd_msg)

    def new_eagle_dbw_publisher(self, event):
        """
        Similar to cmdCallback in JoystickDemo.cpp and JoystickDemo.h
        :param event: rospy.TimerEvent instance passed by the Timer object
        :return:
        """
        # Detect joy timeouts and reset
        self.detect_and_reset_joy_timeout(event)

        # Optional watchdog counter
        if self.count:
            self.counter += 1

            # counter reset/overflow
            if self.counter > 15:
                self.counter = 0

        # Accelerator Pedal
        accelerator_pedal_msg = AcceleratorPedalCmd()
        accelerator_pedal_msg.enable = True  # self.enable
        accelerator_pedal_msg.ignore = self.ignore
        accelerator_pedal_msg.rolling_counter = self.counter
        # for percentage based actuation
        if self.control_type in ('open_loop', 'all'):
            accelerator_pedal_msg.pedal_cmd = self.joystick_data[
                                                  'accelerator_pedal_joy'] * self.MAX_ACCELERATION_ACTUATION * 100
            accelerator_pedal_msg.control_type.value = ActuatorControlMode.open_loop
        # for speed based acceleration
        if self.control_type in ('closed_loop_vehicle', 'all'):
            accelerator_pedal_msg.speed_cmd = self.joystick_data['speed_joy']
            accelerator_pedal_msg.road_slope = 0  # todo: could get this data from the IMU or localization node
            accelerator_pedal_msg.accel_limit = self.joystick_data['accel_decel_limits']
            accelerator_pedal_msg.control_type.value = ActuatorControlMode.closed_loop_vehicle
        self.pub_accelerator_pedal.publish(accelerator_pedal_msg)

        # Brake
        brake_msg = BrakePedalCmd()
        brake_msg.enable = True
        brake_msg.rolling_counter = self.counter
        brake_msg.pedal_cmd = abs(self.joystick_data['brake_joy']) * 100
        # # for percentage based actuation
        if self.control_type in ('open_loop', 'all'):
            brake_msg.control_type.value = ActuatorControlMode.open_loop
        # for speed based acceleration
        if self.control_type in ('closed_loop_vehicle', 'all'):
            brake_msg.control_type.value = ActuatorControlMode.closed_loop_vehicle
            brake_msg.decel_limit = self.joystick_data['accel_decel_limits']
        self.pub_brake.publish(brake_msg)

        # Steering
        steering_msg = SteeringCmd()
        steering_msg.enable = True
        steering_msg.ignore = self.ignore
        steering_msg.rolling_counter = self.counter
        steering_msg.angle_cmd = radians(self.joystick_data["steering_wheel_joy"])
        steering_msg.angle_velocity = self.svel
        steering_msg.control_type.value = ActuatorControlMode.closed_loop_actuator
        self.pub_steering.publish(steering_msg)

        # Gear
        gear_msg = GearCmd()
        gear_msg.enable = True
        gear_msg.cmd.gear = self.joystick_data['gear_cmd']
        gear_msg.rolling_counter = self.counter
        self.pub_gear.publish(gear_msg)

        # Turn Signal
        misc_msg = MiscCmd()
        misc_msg.cmd.value = self.joystick_data['turn_signal_cmd']
        misc_msg.rolling_counter = self.counter
        self.pub_misc.publish(misc_msg)

        # Global Enable
        global_enable_msg = GlobalEnableCmd()
        global_enable_msg.global_enable = True
        global_enable_msg.enable_joystick_limits = True
        global_enable_msg.rolling_counter = self.counter
        self.pub_global_enable.publish(global_enable_msg)

    def detect_and_reset_joy_timeout(self, event, duration=0.1):
        """
        Detects joy timeouts and reset
        :param duration:
        :param event:
        :return:
        """
        if event.current_real - self.joystick_data.get('stamp') > rospy.Duration(duration):
            self.joystick_data['joy_accelerator_pedal_valid'] = False
            self.joystick_data['joy_brake_valid'] = False

    def autonomy_mode_cmd_publisher(self, event):
        # Detect joy timeouts and reset
        self.detect_and_reset_joy_timeout(event)

        autonomy_mode_cmd = Bool()
        autonomy_mode_cmd.data = self.autonomy_mode_status
        self.autonomy_mode_cmd_pub.publish(autonomy_mode_cmd)

    def enable_cmd_publisher(self, event):
        # Detect joy timeouts and reset
        self.detect_and_reset_joy_timeout(event)

        enable_cmd = Bool()
        enable_cmd.data = self.global_enable_status
        self.global_enable_cmd_pub.publish(enable_cmd)

    def disable_cmd_publisher(self, event):
        # Detect joy timeouts and reset
        self.detect_and_reset_joy_timeout(event)

        disable_cmd = Bool()
        disable_cmd.data = self.global_disable_status
        self.global_disable_cmd_pub.publish(disable_cmd)


if __name__ == '__main__':
    print("This module is not meant to be run as a script but imported as a module.")
