import sys
sys.path.append("./")

import os
current_script_path = os.path.abspath(__file__)
project_root = os.path.dirname(os.path.dirname(current_script_path))
dr_path = os.path.join(project_root, "third_party", "dr")
sys.path.insert(0, dr_path)  

from controller.arm_controller import ArmController
import numpy as np
import time

from third_party.dr import aloha_robot as dr# from www.daran.tech

'''
大然aloha机械臂初始化参数
'''
l_p = 150 # 工具参考点到电机输出轴表面的距离，单位mm（所有尺寸参数皆为mm）
l_p_mass_center = 55 # 工具（负载）质心到 6 号关节输出面的距离
G_p = 0.396 # 负载重量，单位kg，所有重量单位皆为kg
uart_baudrate = 115200 # 串口波特率，与CAN模块的串口波特率一致，（出厂默认为 115200，最高460800）
#com = 'COM9' # 在这里输入 COM 端口号
com='/dev/ttyACM0' # 在 jetson nano（ubuntu）下控制机器人，相应的输入连接的串口
# com='/dev/ttyAMA0' # 在树莓派（raspbian）下控制机器人，相应的输入连接的串口
# com='/dev/cu.usbserial-110' # 在苹果电脑 mac 下控制机器人，相应地输入串口
# # 机械臂对象初始化函数函数
# dr = aloha_robot.robot(L_p=l_p, L_p_mass_center=l_p_mass_center, G_p=G_p, com=com, uart_baudrate=uart_baudrate)
angle_list=[1,2,3,4,5,6,7]#电机列表1-7

class DrAlohaController(ArmController):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
        self.com=com
    def set_up(self, com:str):
        draloha=dr.robot(L_p=l_p, L_p_mass_center=l_p_mass_center, G_p=G_p, com=com, uart_baudrate=uart_baudrate)
        '''设置机器人参数'''
        draloha.L = [152, 152, 70, 62]  # Aloha 机械臂尺寸参数列表：[l1, l2, l3, d3]，详见库函数说明
        draloha.G_L = [45, 30, 60, 106, 43, 62]
        ''' G_L 是计算重力补偿时用到的参数
            G_L[0] 是杆件 3 质心到关节 3 轴线在机械臂伸长方向上的距离 G_L_1
            G_L[1] 是关节 4 质心到关节 3 轴线在机械臂伸长方向上的距离 G_L_2
            G_L[2] 是杆件 4 质心到关节 4 端面的距离 G_L_3
            G_L[3] 是关节 5 轴线到关节 4 端面的距离 G_L_4group_readers
            G_L[4] 是杆件 5 质心到关节 5 轴线的距离 G_L_5
            G_L[5] 是关节 6 质到心关节 5 轴线的距离 G_L_6
        '''
        draloha.G = [0.1005, 0.054, 0.057, 0.057, 0.329, 0.183, 0.253, 0.183] + [G_p] # + [G_p] 必不可少
        '''G 是零部件重量 单位 kg
        G[0] 杆件 2 重量 G_Gan_2
        G[1] 杆件 3 重量 G_Gan_3
        G[2] 杆件 4 重量 G_Gan_4group_readers
        G[3] 杆件 5 重量 G_Gan_5
        G[4] 关节 3 重量 G_3DrCanBus
        G[5] 关节 4 重量 G_4
        G[6] 关节 5 重量 G_5
        G[7] 关节 6 重量 G_6
        '''
        draloha.set_pid_joint(1, P=10, I=5, D=0.55)
        draloha.set_pid_joint(2, P=10.56, I=4.95, D=0.39)
        draloha.set_pid_joint(3, P=10.56, I=4.95, D=0.39)
        draloha.set_pid_joint(4, P=10, I=9, D=0.5)
        draloha.set_pid_joint(5,  P=12, I=5, D=0.1)
        draloha.set_pid_joint(6,  P=12, I=5, D=0.096)

        draloha.torque_factors = [1, 0.2, 0.7, 0.5, 1, 0.5] # 于调节模型扭矩与电机扭矩的比例关系，当重力补偿或零力拖动效果不佳时可用该参数调节
        self.controller=draloha
    def reset(self, start_state):
        # 调用set_position或set_joint就行
        pass
    
    # 返回单位为米
    def get_state(self):
        state = {}
        joint = {}
        eef=self.controller.detect_pose()#返回的是x,y,z,第4、5、6关节角
        for i in range(1,7):
            joint[i-1] = self.controller.get_angle(id_num=i)
        gripper=self.controller.detect_wideth_grasp()#读取夹爪张开宽度单位mm
        # print(eef)
        eef=[eef[0][0]*0.001,eef[0][1]*0.001,eef[0][2]*0.001,eef[1][0],eef[1][1],eef[1][2]]
        print(joint)
        joint={
            key:(vlaue/ 180) * 3.1415926
            for key ,vlaue in joint.items()
            }
        state["joint"]=np.array(joint)
        state["qpos"]=np.array(eef)
        state["gripper"]=gripper*0.001*50
        return state
    
    # 单位为米
    def set_position(self, position,theta,speed=10,param=10,mode=1):
        self.controller.set_pose(x_y_z=position,theta_4_5_6=theta,speed=speed,param=param,mode=mode)#运动到指定位置和姿态
        self.controller.pose_done()#等待关节运动到位
    
    def set_joint(self, joint,speed=1.0):
        joint[1]=joint[1]+90#dr Aloha第二个关节电机角度与模型角度有-90度相位差，例如：第二关节逆时针旋转60度，应输入150度，读出无误差
        self.controller.set_joints(angle_list=joint,speed=speed)#控制1~6关节运动
        self.controller.pose_done()#等待关节运动到位
        time.sleep(1)
    
    # 输入的是0~1的张合度
    def set_gripper(self, gripper):
        gripper=int(gripper*50)
        self.controller.grasp(wideth=gripper,speed=10,force=120)
        self.controller.pose_done()#等待关节运动到位
        time.sleep(1)
    
    def __del__(self):
        try:
            if hasattr(self, 'controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass
        #Zero Gravity Model
    def zero_gravity(self):
        self.controller.set_torques(id_list=[1,2,3,4,5,6,7], torque_list=[0, 0, 0, 0, 0, 0, 0], param=0, mode=0) # 设置对应关节扭矩
        angle_list = []
        angle_speed_torque = self.controller.get_angle_speed_torque_all(id_list=[1,2,3,4,5,6,7])
        if angle_speed_torque is None:
            for i in range(4):
                angle_speed_torque = self.controller.get_angle_speed_torque_all(id_list=[1,2,3,4,5,6,7])
                print("angle_speed_torque retry:",i)
                if angle_speed_torque is not None:
                    break
        if angle_speed_torque is None:
            pass
        else:
            for i in range(6):
                angle_list.append(angle_speed_torque[i][0])
            print(angle_list)
            self.controller.gravity_compensation(angle_list=angle_list)
if __name__=="__main__":
    fps=30
    controller=DrAlohaController("test Dr")
    controller.set_up("/dev/ttyACM0")
    init_joint_position = [0.0,60.0, -150.0, 0.0, 0.0, 0.0] 
    controller.set_joint(init_joint_position)
    controller.set_gripper(0)
    state=controller.get_state()
    controller.zero_gravity()
    tele_time=30
    start_time = time.time()
    while True:
        if time.time() - start_time > tele_time:
            break
        state = controller.get_state()
        print(state)
        time.sleep(1/fps)

