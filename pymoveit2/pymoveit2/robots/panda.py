from typing import List

MOVE_GROUP_ARM: str = "ur_manipulator"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "ur_") -> List[str]:
    # return [
    #     prefix + "joint1",
    #     prefix + "joint2",
    #     prefix + "joint3",
    #     prefix + "joint4",
    #     prefix + "joint5",
    #     prefix + "joint6",
    #     prefix + "joint7",
    # ]

    return ["shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "shoulder_pan_joint",]

def base_link_name(prefix: str = "panda_") -> str:
    return "base_link"


def end_effector_name(prefix: str = "panda_") -> str:
    return "wrist_3_link"


def gripper_joint_names(prefix: str = "panda_") -> List[str]:
    return [
        prefix + "finger_joint1",
        prefix + "finger_joint2",
    ]
