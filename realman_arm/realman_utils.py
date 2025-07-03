import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from realman_arm.Robotic_Arm.rm_robot_interface import *
import multiprocessing as mp
import time
import threading
arm_model = rm_robot_arm_model_e.RM_MODEL_RM_65_E  # RM_65机械臂
force_type = rm_force_type_e.RM_MODEL_RM_B_E  # 标准版

# 初始化算法的机械臂及末端型号
algo_handle = Algo(arm_model, force_type) 

# 设置逆解求解模式
algo_handle.rm_algo_set_redundant_parameter_traversal_mode(False)

def rpy_to_rotation_vector(rpy):
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    rotation = R.from_euler('ZYX', [roll, pitch, yaw], degrees=False)
    rotation_vector = rotation.as_rotvec()
    return rotation_vector

def rotation_vector_to_rpy(rotation_vector):
    rotation = R.from_rotvec(rotation_vector)
    rpy = rotation.as_euler('ZYX', degrees=False)
    return rpy

class RobotConfig:
    def __init__(self, config_file="realman_arm/config.yaml"):
        # 加载配置文件
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

    def get_robot_config(self, hand_type):
        return self.config['robot_config'][hand_type]

    def get_feetech_config(self, hand_type):
        return self.config['feetech_config'][hand_type]

class GripperControlThread(threading.Thread):
    def __init__(self, robot, gripper_state, speed, force=None):
        super().__init__()
        self.robot = robot
        self.gripper_state = gripper_state
        self.speed = speed
        self.force = force
        self.daemon = True  # 设置为守护线程，主线程退出时自动结束

    def run(self):
        if self.gripper_state == 1:
            self.robot.rm_set_gripper_pick_on(self.speed, self.force, False, 0)
        else:
            self.robot.rm_set_gripper_release(self.speed, False, 0)

# follower
class Realman:
    def __init__(self, robot_ip, mode_e=True):
        self.config_loader = RobotConfig()
        self.hand_type = "left" if robot_ip == "192.168.1.18" else "right"
        
        self.manager = mp.Manager()
        self.shared = self.manager.dict()
        # 先拿原始 config
        origin_config = self.config_loader.get_robot_config(self.hand_type)

        # 把里面每一个 key 都转成 Manager.dict() 能识别的类型
        shared_config = self.manager.dict()

        for key, value in origin_config.items():
            if isinstance(value, dict):
                shared_config[key] = self.manager.dict(value)
            else:
                shared_config[key] = value

        self.shared[self.hand_type] = shared_config

        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E) if mode_e else RoboticArm()
        self.handle = self.robot.rm_create_robot_arm(self.shared[self.hand_type]["ip"], port=8080, level=3)
        self.arm_state_callback = rm_realtime_arm_state_callback_ptr(self.arm_state_func)
        self.connect()

        if self.handle.id == -1:
            print("\nFailed to connect to the robot arm\n")
            exit(1)
        else:
            print(f"\nSuccessfully connected to the robot arm: {self.handle.id}\n") 

    def init_gripper(self):
        pass

    def connect(self):
        self.udp_watch()

    def arm_state_func(self, data):
        if data.arm_ip.decode() ==  self.shared[self.hand_type]["ip"]:
            waypoint = data.waypoint.to_dict()
            position = waypoint["position"]
            euler = waypoint["euler"]
            self.shared[self.hand_type]["cur_pose"] = [position['x'], position['y'], position['z'], euler["rx"], euler["ry"], euler["rz"]]
            self.shared[self.hand_type]["cur_pos"] = list(data.joint_status.joint_position[:6])
            self.shared[self.hand_type]["cur_speed"] = list(data.joint_status.joint_speed[:6])
            
    def udp_watch(self):
        custom = rm_udp_custom_config_t()
        custom.joint_speed = 1
        custom.lift_state = 0
        custom.expand_state = 0
        custom.arm_current_status = 1
        config = rm_realtime_push_config_t(5, True, 8089, 0, "192.168.1.100", custom)
        self.robot.rm_set_realtime_push(config)
        self.robot.rm_get_realtime_push()
        self.robot.rm_realtime_arm_state_call_back(self.arm_state_callback)

    def getActualTCPPose(self):
        pose = self.get_pose_rot_vec()
        gripper_pos = self.shared[self.hand_type]['gripper']
        return np.append(pose, gripper_pos)

    def getActualQ(self):
        return self.read("Present_Joint")

    def move_to_pose(self, pose_rotvec):
        xyz = pose_rotvec[:3]
        rotvec = pose_rotvec[3:6]
        gripper = pose_rotvec[6] 

        rpy = rotation_vector_to_rpy(rotvec)

        pose = np.concatenate((xyz, rpy))

        self.robot.rm_movel(pose,v=3 ,r=0 ,connect=0,block=1)

        if gripper == 1 and self.shared[self.hand_type]['gripper'] != 1:
            self.shared[self.hand_type]['gripper'] = gripper
            gripper_thread = GripperControlThread(self.robot, gripper, 1000, 600)
            gripper_thread.start()
        if gripper == 0 and self.shared[self.hand_type]['gripper'] != 0:
            self.shared[self.hand_type]['gripper'] = gripper
            gripper_thread = GripperControlThread(self.robot, gripper, 1000)
            gripper_thread.start()
        


    def servo_p(self, pose_rotvec):
        xyz = pose_rotvec[:3]
        rotvec = pose_rotvec[3:6]
        gripper = pose_rotvec[6] 

        rpy = rotation_vector_to_rpy(rotvec)

        pose = np.concatenate((xyz, rpy))

        #逆运动学参数结构体
        params = rm_inverse_kinematics_params_t(self.shared[self.hand_type]["cur_pos"], pose, 1)

        # 计算逆运动学全解(当前仅支持六自由度机器人)
        result = algo_handle.rm_algo_inverse_kinematics_all(params)

        # 从多解中选取最优解(当前仅支持六自由度机器人)
        ret = algo_handle.rm_algo_ikine_select_ik_solve([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], result)
        if ret != -1:
            self.robot.rm_movej_canfd(joint= list(result.q_solve[ret])[:6], follow=True, expand=0, trajectory_mode=2, radio=90)
        else: 
            print("no best solution!")

        if gripper == 1 and self.shared[self.hand_type]['gripper'] != 1:
            self.shared[self.hand_type]['gripper'] = gripper
            gripper_thread = GripperControlThread(self.robot, gripper, 1000, 600)
            gripper_thread.start()
        if gripper == 0 and self.shared[self.hand_type]['gripper'] != 0:
            self.shared[self.hand_type]['gripper'] = gripper
            gripper_thread = GripperControlThread(self.robot, gripper, 1000)
            gripper_thread.start()

    def servo_j(self, joint):
        arm_joint = joint[:6]
        gripper = joint[6]
        code = self.robot.rm_movej_canfd(arm_joint, follow=True, expand=0, trajectory_mode=2, radio=920)
        if gripper == 1 and self.shared[self.hand_type]['gripper'] != 1:
            self.shared[self.hand_type]['gripper'] = gripper
            self.robot.rm_set_gripper_pick_on(1000, 1000, False, 10)
            # self.robot.rm_set_gripper_position(1, False, 10)

        if gripper == 0 and self.shared[self.hand_type]['gripper'] != 0:
            print("松开")
            self.shared[self.hand_type]['gripper'] = gripper
            self.robot.rm_set_gripper_release(1000, False, 10)
        if code != 0:
            print("\nwrite failed, Error code: ", code, "\n")

    def read(self, parameter):
        if parameter == "Present_Position":
            cur_pos = self.shared[self.hand_type]["cur_pose"]
            return np.append(cur_pos, self.shared[self.hand_type]['gripper'])
        if parameter == "Present_Joint":
            cur_joint = self.shared[self.hand_type]["cur_pos"]
            return np.append(cur_joint, self.shared[self.hand_type]['gripper'])

    def get_pose_rot_vec(self):
        pose = self.shared[self.hand_type]["cur_pose"]
        if not pose:
            pose = self.robot.rm_get_current_arm_state()[1]["pose"]
        xyz = pose[:3]
        rpy = pose[3:]
        rot = rpy_to_rotation_vector(rpy)
        xyz_rot = np.concatenate((xyz, rot))
        return xyz_rot
    
    def disconnect(self):
        handle = self.robot.rm_delete_robot_arm()
        if handle == 0:
            print("\nSuccessfully disconnected from the robot arm\n")
        else:
            print("\nFailed to disconnect from the robot arm\n")


