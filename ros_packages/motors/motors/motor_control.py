#ST 
import os
import sys
import serial
#script_dir = os.path.dirname(os.path.abspath(__file__))
#sys.path.insert(0, script_dir)
from motors.STservo_sdk import *


from typing import Iterable, Tuple

import rclpy
from datatypes.msg import MotorSettings
from datatypes.srv import ApplyMotorSettings, ApplyJointTrajectory
from pib_api_client import motor_client
from pib_motors.bricklet import ipcon
from pib_motors.motor import name_to_motors, motors
from pib_motors.update_bricklet_uids import *
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#const values for STS
BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
STS_MINIMUM_POSITION_VALUE  = 0           # STServo will rotate between this value
STS_MAXIMUM_POSITION_VALUE  = 4095
STS_MOVING_SPEED            = 2400        # STServo moving speed
STS_MOVING_ACC              = 50 

motor_map = {
            "thumb_right_opposition": 1,
            "thumb_right_stretch": 1,
            "index_right_stretch": 12,
            "middle_right_stretch": 13,
            "ring_right_stretch": 14,
            "pinky_right_stretch": 15,
            "wrist_right": 16,
            "lower_arm_right_rotation": 17,
            "elbow_right": 18,
            "upper_arm_right_rotation": 19,
            "shoulder_horizontal_right": 20,
            "shoulder_vertical_right": 21,
            # IDs 22, 23 missing?
            "turn_head_motor": 24,
            "tilt_forward_motor": 25,
            # IDs 26, 27 missing?
            "shoulder_horizontal_left": 28,
            "shoulder_vertical_left": 29,
            "thumb_left_opposition": 30,
            "thumb_left_stretch": 31,
            "index_left_stretch": 32,
            "middle_left_stretch": 33,
            "ring_left_stretch": 34,
            "pinky_left_stretch": 35,
            "wrist_left": 36,
            "lower_arm_left_rotation": 37,
            "elbow_left": 38,
            "upper_arm_left_rotation": 39,
        }

portHandler = port_handler.PortHandler(DEVICENAME)
packetHandler = sts(portHandler)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

def motor_settings_ros_to_dto(ms: MotorSettings):
    return {
        "name": ms.motor_name,
        "turnedOn": ms.turned_on,
        "pulseWidthMin": ms.pulse_width_min,
        "pulseWidthMax": ms.pulse_width_max,
        "rotationRangeMin": ms.rotation_range_min,
        "rotationRangeMax": ms.rotation_range_max,
        "velocity": ms.velocity,
        "acceleration": ms.acceleration,
        "deceleration": ms.deceleration,
        "period": ms.period,
        "visible": ms.visible,
        "invert": ms.invert,
    }


def as_motor_positions(jt: JointTrajectory) -> Iterable[Tuple[str, int]]:
    """unpacks a jt-message into an iterable of motorname-position-pairs"""
    motor_names = jt.joint_names
    points: list[JointTrajectoryPoint] = jt.points
    positions = (point.positions[0] for point in points)
    return zip(motor_names, positions)


def as_joint_trajectory(motor_name: str, position: int) -> JointTrajectory:
    """converts a motorname and position into a simple jt-message"""
    jt = JointTrajectory()
    jt.joint_names = [motor_name]
    point = JointTrajectoryPoint()
    point.positions.append(position)
    jt.points = [point]
    return jt


class MotorControl(Node):

    def __init__(self):

        super().__init__("motor_control")

        # Toggle Devmode
        self.declare_parameter("dev", False)
        self.dev = self.get_parameter("dev").value

        # Service for JointTrajectory
        self.srv = self.create_service(
            ApplyJointTrajectory, "apply_joint_trajectory", self.apply_joint_trajectory
        )

        # Publisher for JointTrajectory
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, "joint_trajectory", 10
        )

        # Service for MotorSettings
        self.srv = self.create_service(
            ApplyMotorSettings, "apply_motor_settings", self.apply_motor_settings
        )

        # Publisher for MotorSettings
        self.motor_settings_publisher = self.create_publisher(
            MotorSettings, "motor_settings", 10
        )

        # load motor-settings if not in dev mode
        if not self.dev:
            for motor in motors:
                if motor.check_if_motor_is_connected():
                    successful, motor_settings_dto = motor_client.get_motor_settings(
                        motor.name
                    )
                    if successful:
                        motor.apply_settings(motor_settings_dto)

        # Log that initialization is complete
        self.get_logger().info("Now Running MOTOR_CONTROL")

    def apply_motor_settings(
        self, request: ApplyMotorSettings.Request, response: ApplyMotorSettings.Response
    ) -> ApplyMotorSettings.Response:

        response.settings_applied = True
        response.settings_persisted = True

        motor_settings_ros = request.motor_settings
        motor_settings_dto = motor_settings_ros_to_dto(motor_settings_ros)

        try:
            motors = name_to_motors[request.motor_settings.motor_name]
            for motor in motors:
                motor_settings_dto["name"] = motor.name
                motor_settings_ros.motor_name = motor.name
                applied = motor.apply_settings(motor_settings_dto)
                response.settings_applied &= applied
                if applied or self.dev:
                    persisted, _ = motor_client.update_motor_settings(
                        motor.name, motor_settings_dto
                    )
                    response.settings_persisted &= persisted
                    self.motor_settings_publisher.publish(motor_settings_ros)
                self.get_logger().info(f"updated motor: {str(motor)}")

        except Exception as e:
            response.settings_applied = False
            response.settings_persisted = False
            self.get_logger().warn(
                f"Error while processing motor-settings-message: {str(e)}"
            )

        return response

    def apply_joint_trajectory(
        self,
        request: ApplyJointTrajectory.Request,
        response: ApplyJointTrajectory.Response,
    ) -> ApplyJointTrajectory.Response:
        jt = request.joint_trajectory
        response.successful = True
        try:
            for motor_name, position in as_motor_positions(jt):
                for motor in name_to_motors[motor_name]:
                    self.get_logger().info(
                        f"setting position of {motor.name} to {position}"
                    )
                    
                    #successful = motor.set_position(position)
                    sts_goal_position =((position+9000)*0.23)
                    STS_ID=motor_map.get(motor.name)
                    packetHandler.WritePosEx(STS_ID, int(sts_goal_position), STS_MOVING_SPEED, STS_MOVING_ACC)
                    #self.get_logger().info(
                    #    f"setting position {'succeeded' if successful else 'failed'}."
                    #)
                    #response.successful &= successful
                    self.joint_trajectory_publisher.publish(
                        as_joint_trajectory(motor.name, position)
                    )
        except Exception as e:
            response.successful = False
            self.get_logger().error(f"error while applying joint-trajectory: {str(e)}")
        return response


def main(args=None):

    rclpy.init(args=args)
    motor_control = MotorControl()
    rclpy.spin(motor_control)
    rclpy.shutdown()
    ipcon.disconnect()


if __name__ == "__main__":
    main()
