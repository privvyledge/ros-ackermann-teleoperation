#!/usr/bin/env python3

"""
This script takes in  a sensor_msgs/Joy message from the Logitech G29 racing cockpit and outputs either
    (1) AckermannDriveStamped (2) Autoware's Control Command Stamped message or (3) Twist message and
    optionally New Eagle messages. Could be used for simulations as well as hardware.

See: https://github.com/autowarefoundation/autoware.universe/tree/main/control/joy_controller for Autoware example.

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

__author__ = 'Boluwatife Olabiran'
__license__ = 'GPLv3'
__maintainer__ = 'Boluwatife Olabiran'
__email__ = 'bso19a@fsu.edu'


class RacingCockpitTeleopNode(object):
    """docstring for ClassName"""

    def __init__(self, ):
        """Constructor for RacingCockpitTeleopNode"""
        super(RacingCockpitTeleopNode, self).__init__()

        # get parameters (max speed, steering, acceleration)
        self.MAX_SPEED = float(rospy.get_param('~max_speed', 10.0))  # m/s
        self.MAX_STEERING_ANGLE = float(rospy.get_param('~max_steering_angle', 0.49))
        self.MAX_STEERING_WHEEL_ANGLE = int(rospy.get_param('~max_steering_wheel_angle', 470))
        self.MAX_ACCELERATION = float(rospy.get_param('~max_acceleration', 2.0))
        self.MAX_ACCELERATION_ACTUATION = float(rospy.get_param('~max_acceleration_actuation', 1))
        self.MAX_BRAKE_ACTUATION = float(rospy.get_param('~max_brake_actuation', 1))
        # NOTE 1 MPH = 0.44704 m/s and 1 m/s = 2.23694 MPH
        # amount to raise or decrease the speed by. Translates to 1 m/s
        self.SPEED_INCREMENT = rospy.get_param('~speed_increment', 0.44704)  # in m/s.
        # message publish type
        self.pub_mode = rospy.get_param('~message_pub_mode', 'vehicle_cmd')  # ackermann, twist, vehicle_cmd, or all
        self.pub_dbw_message = rospy.get_param('~pub_dbw_message', True)
        self.stamped = rospy.get_param('~stamp_messages', True)  # publish the stamped version of messages
        # safety commands
        self.ignore = rospy.get_param('~ignore', False)  # Ignore driver overrides
        self.enable = rospy.get_param('~enable', True)  # Use enable and disable buttons
        # ToDo: implement rolling counter with message publisher callbacks
        self.count = rospy.get_param('~count',
                                     False)  # Increment counter to enable watchdog.
        self.svel = rospy.get_param('~svel', 0.0)  # Steering command speed
        self.global_enable_status = rospy.get_param('~global_enable_status', False)
        self.global_disable_status = rospy.get_param('~global_disable_status', False)
        self.clip_steering_angle = rospy.get_param('~clip_steering_angle', False)
        self.control_types = dict(open_loop=0, closed_loop_actuator=1, closed_loop_vehicle=2, none=255)
        self.control_type = rospy.get_param('~control_type', 'closed_loop_vehicle')
        # car parameters
        self.WHEELBASE = 1.0  # so downstream nodes can modify based on theirs. ToDo specify in documentation

        # joystick data. ToDO: could make into a message
        self.joystick_data = {'stamp': rospy.Time.now(),
                              'brake_joy': 0.0,
                              'accelerator_pedal_joy': 0.0,
                              'abs_speed_joy': 0.0,
                              'speed_joy': 0.0,
                              'steering_joy': 0.0,
                              'steering_wheel_joy': 0.0,
                              'steering_mult': False,
                              'gear_cmd': 0,
                              'turn_signal_cmd': 0,
                              'joy_accelerator_pedal_valid': False,
                              'joy_brake_valid': False,
                              'accel_decel_limits': 3.0}

        # joystick button to index mapping. todo: missing two buttons in the middle
        # buttons
        self.X_BUTTON = 2  # left
        self.Y_BUTTON = 3  # top
        self.B_BUTTON = 1  # right
        self.A_BUTTON = 0  # down
        self.PADDLE_RIGHT = 4
        self.PADDLE_LEFT = 5
        self.DOUBLE_BOX_BUTTON = 6  # probably double box button. ToDo: check
        self.HAMBURGER_BUTTON = 7  # probably hamburger menu button. ToDo: check
        self.LSB_BUTTON = 8
        self.RSB_BUTTON = 9
        self.XBOX_BUTTON = 10
        self.GEAR_LEFT_FRONT = 12
        self.GEAR_MIDDLE_FRONT = 14
        self.GEAR_RIGHT_FRONT = 16
        self.GEAR_LEFT_BACK = 13
        self.GEAR_MIDDLE_BACK = 15
        self.GEAR_RIGHT_BACK = 17
        # axes
        self.STEERING_WHEEL_AXIS = 0  # +: left, -: right [-1, 1]
        self.THROTTLE_AXIS = 1  # [-1, 1]
        self.BRAKE_AXIS = 2
        self.CLUTCH_AXIS = 3
        self.DIRECTION_SIDEWAYS = 4  # left>0.0, right<0.0
        self.DIRECTION_LONGITUDINAL = 5  # up>0.0, down<0.0

        # joystick button to vehicle mapping
        self.BTN_PARK = self.Y_BUTTON
        self.BTN_REVERSE = self.B_BUTTON
        self.BTN_NEUTRAL = self.X_BUTTON
        self.BTN_DRIVE = self.A_BUTTON
        self.BTN_ENABLE = self.RSB_BUTTON
        self.BTN_DISABLE = self.LSB_BUTTON
        self.BTN_STEER_MULT_1 = self.HAMBURGER_BUTTON  # start button
        self.BTN_ENABLE_AUTONOMOUS_MODE = self.DOUBLE_BOX_BUTTON  # back button
        self.AXIS_ACCELERATOR_PEDAL = self.THROTTLE_AXIS
        self.AXIS_BRAKE = self.BRAKE_AXIS
        self.AXIS_STEER_1 = self.STEERING_WHEEL_AXIS
        self.AXIS_TURN_SIG = self.DIRECTION_SIDEWAYS
        self.AXIS_SPEED_INCREMENT = self.DIRECTION_LONGITUDINAL  # or could use paddle shifters

        # joystick counts
        self.BTN_COUNT = 18
        self.AXIS_COUNT = 6

        # to get the current ROS time. Use this function call as it works both for simulation and wall time.
        self.current_time = rospy.Time.now()
        self.counter = 0

        # vehicle actuation modes
        '''
        self.unify_speed_increment_and_pedals means the speed from the directional increment and pedals will be 
        seamless. The default should be True. False means whenever any of them is first used to set the speed, if the 
        other option is used it will start from zero which might lead to stopping instantly and jerking forward.
        '''
        self.unify_speed_increment_and_pedals = rospy.get_param('~unify_speed_increment_and_pedals', False)
        '''
        self.coast_mode = True means the car will keep on moving in the direction of the gear with the last speed and 
        will only come to a stop if brake is pressed or based on the deceleration/kinematic profile of the car to slow
        down. False means speed will decrease as accelerator pedal is gradually released. This mode is only useful if 
        self.unify_speed_increment_and_pedals = False
        '''
        self.coast_mode = rospy.get_param('~coast_mode', False)
        self.set_speed = 0.0  # the last speed which will be maintained/decreased
        '''
        self.mirror_speed_with_gear_command emulates gears with the speed commands, e.g if gear is set to Park, speed
        will be zero even if the accelerator is pressed, if gear is set to Reverse, speed will be negative, if gear is
        set to Drive, speed will be positive. The gears state will also change even if this is on so could serve as 
        redundancy for cars with gear selectors.
        '''
        self.mirror_speed_with_gear_command = rospy.get_param('~mirror_speed_with_gear_command', True)

        # miscellaneous
        self.empty = Empty()
        self.joy = Joy()
        self.joy.axes = [0] * self.AXIS_COUNT
        self.joy.buttons = [0] * self.BTN_COUNT
        self.print_instructions()
        self.gear_dict = {0: 'None', 1: 'P', 2: 'R', 3: 'N', 4: 'D', 5: 'L'}
        self.normalized_steering_angle = 1 / self.MAX_STEERING_ANGLE  # normalizes the steering angle to [-1, 1]

        # temporary variables (to fix triggers not starting properly)
        self.RT_press_count = False
        self.LT_press_count = False

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = 50.0  # Hz
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)

        # setup threads to run asynchronously
        period = 0.02  # a.k.a. duration in s (or 1/Hz)

        # one thread running at a low frequency to act as a watchdog
        self.timer_watchdog = rospy.Timer(rospy.Duration(2), self.watchdog, reset=True)

        # instantiate subscribers
        subscriber_queue_size = 1
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=subscriber_queue_size)

        # instantiate publishers
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

        self.autonomy_mode_status = False
        self.autonomy_mode_cmd_pub = rospy.Publisher("autonomy_mode", Bool, queue_size=publisher_queue_size)
        rospy.Timer(rospy.Duration(period), self.autonomy_mode_cmd_publisher,
                    oneshot=False, reset=True)

    def print_instructions(self):
        """
        Print instructions/joystick buttons and modes.
        :return:
        """
        pass

    def joystick_callback(self, msg):
        """
        :param msg:
        :return:
        """
        # Check for expected sizes
        if len(msg.axes) != self.AXIS_COUNT:
            rospy.logerr("Expected {} joy axis count, received {}".format(self.AXIS_COUNT, len(msg.axes)))
            return

        if len(msg.buttons) != self.BTN_COUNT:
            rospy.logerr("Expected {} joy button count, received {}".format(self.BTN_COUNT, len(msg.buttons)))
            return

        if msg.buttons[self.BTN_ENABLE_AUTONOMOUS_MODE]:
            self.autonomy_mode_status = not self.autonomy_mode_status

        # Handle check if RT and LT have been pressed as well as handle joystick startup
        if msg.axes[self.AXIS_ACCELERATOR_PEDAL] != 0.0:
            if not self.RT_press_count:
                self.RT_press_count = True

        if msg.axes[self.AXIS_BRAKE] != 0.0:
            if not self.LT_press_count:
                self.LT_press_count = True

        if msg.axes[self.AXIS_ACCELERATOR_PEDAL] <= 0.95 and self.RT_press_count != 0:
            self.joystick_data["joy_accelerator_pedal_valid"] = True

        if msg.axes[self.AXIS_BRAKE] <= 0.95 and self.LT_press_count != 0:
            self.joystick_data["joy_brake_valid"] = True

        # Acceleration [0, 1].
        desired_acceleration = self.set_speed
        last_accelerator_pedal_cmd = self.joystick_data["accelerator_pedal_joy"]
        if self.joystick_data["joy_accelerator_pedal_valid"]:
            current_accelerator_pedal_cmd = 0.5 - 0.5 * msg.axes[self.AXIS_ACCELERATOR_PEDAL]
            if self.coast_mode:
                self.joystick_data["accelerator_pedal_joy"] = max(abs(desired_acceleration),
                                                                  current_accelerator_pedal_cmd)
            else:
                self.joystick_data["accelerator_pedal_joy"] = \
                    self.MAX_ACCELERATION_ACTUATION * current_accelerator_pedal_cmd  # [0, 1]

            desired_acceleration = self.joystick_data["accelerator_pedal_joy"]

        # Brake [-1, 0].
        desired_brake = 0.0
        if self.joystick_data["joy_brake_valid"]:
            self.joystick_data["brake_joy"] = \
                -self.MAX_BRAKE_ACTUATION * (0.5 - 0.5 * msg.axes[self.AXIS_BRAKE])  # [-1, 0]
            desired_brake = self.joystick_data["brake_joy"]

        if self.joystick_data["joy_accelerator_pedal_valid"] or self.joystick_data["joy_brake_valid"]:
            # todo: currently goes beyond max speed
            if self.unify_speed_increment_and_pedals:
                self.joystick_data['abs_speed_joy'] += (desired_acceleration + desired_brake) * self.MAX_SPEED
            else:
                self.joystick_data['abs_speed_joy'] = (desired_acceleration + desired_brake) * self.MAX_SPEED
            self.joystick_data['abs_speed_joy'] *= self.SPEED_INCREMENT

        # Speed increment.
        if msg.axes[7] != self.joy.axes[7]:
            if msg.axes[7] < -0.5:
                self.joystick_data['abs_speed_joy'] -= self.SPEED_INCREMENT
                if self.joystick_data['abs_speed_joy'] < 0:
                    self.joystick_data['abs_speed_joy'] = 0.0
            elif msg.axes[7] > 0.5:
                self.joystick_data['abs_speed_joy'] += self.SPEED_INCREMENT
                if self.joystick_data['abs_speed_joy'] > self.MAX_SPEED * self.SPEED_INCREMENT:
                    self.joystick_data['abs_speed_joy'] = self.MAX_SPEED * self.SPEED_INCREMENT

        try:
            speed_sign = self.joystick_data['abs_speed_joy'] / abs(self.joystick_data['abs_speed_joy'])
        except ZeroDivisionError:
            speed_sign = 1

        self.joystick_data['abs_speed_joy'] = speed_sign * abs(min(self.joystick_data['abs_speed_joy'], self.MAX_SPEED))
        self.joystick_data['abs_speed_joy'] = max(self.joystick_data['abs_speed_joy'], 0.0)
        self.set_speed = self.joystick_data['abs_speed_joy'] / self.MAX_SPEED

        # switch gears using a State Machine
        last_gear = self.joystick_data["gear_cmd"]
        new_gear_flag, new_gear_cmd = self.fetch_new_gear(msg)

        if abs(self.joystick_data["brake_joy"]) >= (self.MAX_BRAKE_ACTUATION - 0.2)\
                and not self.joystick_data['abs_speed_joy']:
            new_gear_flag = new_gear_flag and True
        else:
            new_gear_flag = new_gear_flag and False
            new_gear_cmd = last_gear

        def park(data, gear_flag, gear_cmd):
            if gear_flag:
                # change will be applied in next time step to prevent race conditions
                self.joystick_data["gear_cmd"] = gear_cmd
            else:
                self.joystick_data['speed_joy'] = data
                if self.mirror_speed_with_gear_command:
                    self.joystick_data['speed_joy'] = 0.0
            return

        def reverse(data, gear_flag, gear_cmd):
            if gear_flag:
                self.joystick_data["gear_cmd"] = gear_cmd
            else:
                self.joystick_data['speed_joy'] = data
                if self.mirror_speed_with_gear_command:
                    self.joystick_data['speed_joy'] = -data
            return

        def drive(data, gear_flag, gear_cmd):
            if gear_flag:
                self.joystick_data["gear_cmd"] = gear_cmd
            else:
                self.joystick_data['speed_joy'] = data
                if self.mirror_speed_with_gear_command:
                    self.joystick_data['speed_joy'] = abs(data)
            return

        def neutral(data, gear_flag, gear_cmd):
            if gear_flag:
                self.joystick_data["gear_cmd"] = gear_cmd
            else:
                # pass
                if self.mirror_speed_with_gear_command:
                    pass
            return

        def low_gear(data, gear_flag, gear_cmd):
            if gear_flag:
                self.joystick_data["gear_cmd"] = gear_cmd
            else:
                rospy.loginfo("No gear specified. Using value of {} instead.".format(self.gear_dict[0]))
            return

        def default(data, gear_flag, gear_cmd):
            if gear_flag:
                self.joystick_data["gear_cmd"] = gear_cmd
            else:
                rospy.loginfo("No gear specified. Using None value of {} instead.".format(0))
                if not self.mirror_speed_with_gear_command:
                    self.joystick_data['speed_joy'] = data
            return

        # {0: None, 1: 'P', 2: 'R', 3: 'N', 4: 'D', 5: 'L'}
        gear_switcher = {0: default, 1: park, 2: reverse, 3: neutral, 4: drive, 5: low_gear}
        gear_switcher.get(last_gear, default)(abs(self.joystick_data['abs_speed_joy']), new_gear_flag, new_gear_cmd)

        # Steering [-self.self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE] in rads
        '''
        Downstream nodes that need steering wheel angle should convert from this.
        '''
        steering_joy = msg.axes[self.AXIS_STEER_1] * self.MAX_STEERING_ANGLE
        normalized_steering = self.normalized_steering_angle * steering_joy
        self.joystick_data['steering_joy'] = steering_joy
        # self.MAX_STEERING_WHEEL_ANGLE * normalized_steering assumes a linear (not affine) relationship
        # between the steering angle and steering wheel angle which might not be the case
        self.joystick_data['steering_wheel_joy'] = self.MAX_STEERING_WHEEL_ANGLE * normalized_steering  # in degrees
        self.joystick_data["steering_mult"] = msg.buttons[self.BTN_STEER_MULT_1]
        # for safety limit maximum steering wheel angle if steering_mult is not pressed
        if not self.joystick_data['steering_mult']:
            self.joystick_data["steering_wheel_joy"] *= 0.5
            if self.clip_steering_angle:
                self.joystick_data['steering_joy'] *= 0.5

        # Turn Signals. ToDo: create a ROS message for turn signals
        if msg.axes[self.AXIS_TURN_SIG] != self.joy.axes[self.AXIS_TURN_SIG]:
            # no turn signal: 0, left turn signal: 1, right: 2, hazard/both: 3
            def turn_signal_none(data):
                if data.axes[self.AXIS_TURN_SIG] < -0.5:
                    self.joystick_data["turn_signal_cmd"] = 2
                elif data.axes[self.AXIS_TURN_SIG] > 0.5:
                    self.joystick_data["turn_signal_cmd"] = 1
                return

            def turn_signal_left(data):
                if data.axes[self.AXIS_TURN_SIG] < -0.5:
                    self.joystick_data["turn_signal_cmd"] = 2
                elif data.axes[self.AXIS_TURN_SIG] > 0.5:
                    self.joystick_data["turn_signal_cmd"] = 0
                return

            def turn_signal_right(data):
                if data.axes[self.AXIS_TURN_SIG] < -0.5:
                    self.joystick_data["turn_signal_cmd"] = 0
                elif data.axes[self.AXIS_TURN_SIG] > 0.5:
                    self.joystick_data["turn_signal_cmd"] = 1
                return

            def default(data):
                print("Invalid option. Using default value of {} instead.".format(0))
                self.joystick_data["turn_signal_cmd"] = 0
                return

            turn_signal_switcher = {0: turn_signal_none, 1: turn_signal_left, 2: turn_signal_right}
            turn_signal_switcher.get(self.joystick_data["turn_signal_cmd"], default)(msg)

        # Optional enable and disable buttons
        if self.enable:
            if msg.buttons[self.BTN_ENABLE]:
                self.pub_enable.publish(self.empty)
                self.global_enable_status = not self.global_enable_status
            if msg.buttons[self.BTN_DISABLE]:
                self.pub_disable.publish(self.empty)
                self.global_disable_status = not self.global_disable_status

        self.joystick_data['stamp'] = rospy.Time.now()
        self.joy = msg

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
            accelerator_pedal_msg.pedal_cmd = self.joystick_data['accelerator_pedal_joy'] * self.MAX_ACCELERATION_ACTUATION * 100
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

    def fetch_new_gear(self, msg):
        new_gear_flag = False
        if msg.buttons[self.BTN_PARK]:
            gear_cmd = 1
            new_gear_flag = True
        elif msg.buttons[self.BTN_REVERSE]:
            gear_cmd = 2
            new_gear_flag = True
        elif msg.buttons[self.BTN_DRIVE]:
            gear_cmd = 4
            new_gear_flag = True
        elif msg.buttons[self.BTN_NEUTRAL]:
            gear_cmd = 3
            new_gear_flag = True
        else:
            gear_cmd = 0  # this is used for None gear. Could replace with last_gear_cmd
        return new_gear_flag, gear_cmd

    def convert_ackermann_steering_angle_to_yaw_rate(self, speed, steering_angle):
        """

        :param speed:
        :param steering_angle:
        :return:
        """
        tan_steering_angle = tan(steering_angle)
        radius = self.WHEELBASE * tan_steering_angle  # should use wheelbase=1

        if tan_steering_angle == 0.0:
            omega = 0
            return omega

        # # method 1: if wheelbase is specified. Don't use so joystick can be independent of a specific car
        # omega = speed / radius

        # method 2: assumes wheelbase is 1 so downstream nodes can correct based on their wheelbase,
        # i.e omega = omega / wheelbase
        omega = speed / tan_steering_angle
        return omega

    def watchdog(self, event):
        """
        This callback is called, e.g. to monitor a topic, parameter, notify the user that a subscriber does is not
        publishing instead of using a rospy.wait_for_message since that is thread blocking.
        Should run at a low frequency.
        :param event:
        :return:
        """
        pass

    def shutdown(self):
        rospy.loginfo("Beginning clean shutdown routine...")
        # perform shutdown tasks here
        self.joystick_data = {'stamp': rospy.Time.now(),
                              'brake_joy': 0.0,
                              'accelerator_pedal_joy': 0.0,
                              'abs_speed_joy': 0.0,
                              'speed_joy': 0.0,
                              'steering_joy': 0.0,
                              'steering_wheel_joy': 0.0,
                              'steering_mult': False,
                              'gear_cmd': None,
                              'turn_signal_cmd': None,
                              'joy_accelerator_pedal_valid': False,
                              'joy_brake_valid': False,
                              'accel_decel_limits': 3.0}
        # todo: shut down timer objects
        rospy.loginfo("Shutting down...")


def main(args):
    # args will be a list of commands passed
    optional_nodename = 'joystick_teleoperation_node'
    rospy.init_node('{}'.format(optional_nodename))
    nodename = rospy.get_name()  # this gets the actual nodename whether the node is launched or run separately
    rospy.loginfo("{} node started.".format(nodename))  # just log that the node has started.
    joystick_teleop_instance = RacingCockpitTeleopNode()

    # use rospy.on_shutdown() to perform clean shutdown tasks, e.g. saving a file, shutting down motors, etc.
    rospy.on_shutdown(joystick_teleop_instance.shutdown)
    # could also use atexit.register(sample_instance.shutdown) to avoid trusting rospy

    try:
        # run the main functions here
        # joystick_teleop_instance.run()
        rospy.spin()  # only necessary if not publishing (i.e. subscribing only)
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        # this exception block most likely won't be called since there is a shutdown method in the class that will
        # override this and shutdown however is needed but is here just in case.
        rospy.loginfo('Encountered {}. Shutting down.'.format(e))

        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)  # ROS compatible way to handle command line arguments, i.e main(sys.argv)
