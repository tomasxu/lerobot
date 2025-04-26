# read motor steps given motor id   

import numpy as np
import json
from pathlib import Path
from lerobot.common.robot_devices.motors.utils import MotorsBus, make_motors_buses_from_configs
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig
from lerobot.common.robot_devices.motors.feetech import TorqueMode
from lerobot.common.robot_devices.robots.utils import get_arm_id
from typing import Literal

def read_motor_steps(motor_bus: MotorsBus, motor_id: int, arm_type: Literal["leader", "follower"] = "follower") -> int:
    """
    Read the current position (steps) of a motor given its ID.
    
    Args:
        motor_bus: The motor bus interface to communicate with the motor
        motor_id: The ID of the motor to read
        arm_type: The type of arm, either "leader" or "follower"
        
    Returns:
        int: The current position of the motor in steps
        
    Example usage for SO-100's Feetech motor:
    ```python
    # 1. 创建电机总线实例
    motor_bus = FeetechMotorsBus()  # 使用默认配置
    
    # 2. 连接电机总线
    motor_bus.connect()
    
    # 3. 读取电机位置
    # 读取 follower arm 的电机位置
    position = read_motor_steps(motor_bus, 1)  # 默认是 follower arm
    print(f"Follower arm motor position: {position} steps")
    
    # 读取 leader arm 的电机位置
    position = read_motor_steps(motor_bus, 1, arm_type="leader")
    print(f"Leader arm motor position: {position} steps")
    
    # 4. 使用完后断开连接
    motor_bus.disconnect()
    ```
    """
    if not motor_bus.is_connected:
        raise RuntimeError("Motor bus is not connected")
        
    # Read the present position of the motor directly
    # Find motor name from id in motor_bus.motors
    motor_name = None
    for name, (idx, _) in motor_bus.motors.items():
        if idx == motor_id:
            motor_name = name
            break
    if motor_name is None:
        raise ValueError(f"No motor found with ID {motor_id}")
    
    present_position = motor_bus.read("Present_Position", motor_name)
    present_position = int(present_position[0])

        
    return int(present_position)

def read_all_motors_steps(motor_bus: MotorsBus, arm_type: Literal["leader", "follower"] = "follower") -> dict:
    """
    Read the current position (steps) of all motors.
    
    Args:
        motor_bus: The motor bus interface to communicate with the motors
        arm_type: The type of arm, either "leader" or "follower"
        
    Returns:
        dict: A dictionary mapping motor names to their current positions
    """
    if not motor_bus.is_connected:
        raise RuntimeError("Motor bus is not connected")
        
    positions = {}
    for motor_name, (motor_id, _) in motor_bus.motors.items():
        present_position = motor_bus.read("Present_Position", motor_name)
        positions[motor_name] = int(present_position[0])
    
    return positions

if __name__ == "__main__":
    import argparse
    from lerobot.common.robot_devices.robots.configs import So100RobotConfig
    
    parser = argparse.ArgumentParser(description="Read all motor steps for SO-100's Feetech motors")
    parser.add_argument("--arm-type", type=str, choices=["leader", "follower"], default="follower",
                       help="Type of arm (leader or follower)")
    
    args = parser.parse_args()
    
    # 创建机器人配置
    config = So100RobotConfig()
    
    # 根据arm_type选择对应的电机配置
    if args.arm_type == "leader":
        motor_configs = config.leader_arms
    else:
        motor_configs = config.follower_arms
    
    # 创建电机总线
    motor_buses = make_motors_buses_from_configs(motor_configs)
    
    # 连接所有电机总线
    for bus in motor_buses.values():
        bus.connect()
        # Load calibration from file
        arm_id = get_arm_id("main", args.arm_type)
        calib_path = Path(config.calibration_dir) / f"{arm_id}.json"
        if calib_path.exists():
            with open(calib_path) as f:
                calibration = json.load(f)
            bus.set_calibration(calibration)
        else:
            print(f"Warning: Calibration file {calib_path} not found. Using default calibration.")
    
    try:
        # 读取所有电机位置
        positions = read_all_motors_steps(motor_buses["main"], args.arm_type)
        print(f"Current positions of {args.arm_type} arm motors:")
        for motor_name, position in positions.items():
            print(f"  {motor_name:<15} {position:>8} steps")
    finally:
        # 确保断开所有电机总线的连接
        for bus in motor_buses.values():
            bus.disconnect()







