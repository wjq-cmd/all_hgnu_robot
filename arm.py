#!/usr/bin/env python3
#coding=utf-8

import rospy
import json
import serial
import time
from std_msgs.msg import Int32, String, Bool
from std_msgs.msg import Float32MultiArray

class ArmAction:
    """机械臂动作类，用于定义单个动作的参数"""
    def __init__(self, base, shoulder, elbow, wrist, claw, duration):
        self.base = base
        self.shoulder = shoulder
        self.elbow = elbow
        self.wrist = wrist
        self.claw = claw
        self.duration = duration

# 从arm.py导入的动作序列
arm_sequence_area1_0 = [ #左右上中下   # 12,first is right,second is opposite,third is -90,fourth is null,fifth is -45-->20
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  0, 34, 58, 27, 0.1),
    ArmAction(   -90,  7, 25, 58, 27, 6.0),#最上面
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  0, 12, 65, 27, 6.0),#中间
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  -20, 34, 85, 27, 1.5),
    ArmAction(   -90,  -20, -35, 85, 27, 1.5),
    ArmAction(   -90,  -20, -35, 15, 27, 6.0),#最下面
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  0, 34, 58, 27, 0.1),
    ArmAction(   90,  7, 25, 58, 27, 6.0),#最上面
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  0, 12, 65, 27, 6.0),#中间
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  -20, 34, 85, 27, 1.5),
    ArmAction(   90,  -20, -35, 85, 27, 1.5),
    ArmAction(   90,  -20, -35, 15, 27, 6.0),#最下面

]

arm_sequence_area1_1 = [ #左上下右中,5
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  0, 34, 58, 27, 0.1),
    ArmAction(   -90,  7, 25, 58, 27, 6.0),#最上面
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  -20, 34, 85, 27, 1.5),
    ArmAction(   -90,  -20, -35, 85, 27, 1.5),
    ArmAction(   -90,  -20, -35, 15, 27, 6.0),#最下面
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  0, 12, 65, 27, 6.0),#中间
    ArmAction(   0,  0, 0, 0, 27, 6.0),
]

arm_sequence_area1_2 = [ # 左中右上下,6
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  0, 12, 65, 27, 6.0),#中间
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  0, 34, 58, 27, 0.1),
    ArmAction(   90,  7, 25, 58, 27, 6.0),#最上面
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  -20, 34, 85, 27, 1.5),
    ArmAction(   90,  -20, -35, 85, 27, 1.5),
    ArmAction(   90,  -20, -35, 15, 27, 6.0),#最下面
    ArmAction(   0,  0, 0, 0, 27, 6.0),
]

arm_sequence_area1_3 = [ #左上下右上下,7
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  0, 34, 58, 27, 0.1),
    ArmAction(   -90,  7, 25, 58, 27, 6.0),#最上面
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  -20, 34, 85, 27, 1.5),
    ArmAction(   -90,  -20, -35, 85, 27, 1.5),
    ArmAction(   -90,  -20, -35, 15, 27, 6.0),#最下面
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  0, 34, 58, 27, 0.1),
    ArmAction(   90,  7, 25, 58, 27, 6.0),#最上面
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  -20, 34, 85, 27, 1.5),
    ArmAction(   90,  -20, -35, 85, 27, 1.5),
    ArmAction(   90,  -20, -35, 15, 27, 6.0),#最下面
    ArmAction(   0,  0, 0, 0, 27, 6.0),
]

arm_sequence_area1_4 = [ #左中右中,8
    ArmAction(   90,  0, 34, 85, 27, 6.0),
    ArmAction(   90,  0, 12, 65, 27, 6.0),#中间
    ArmAction(   -90,  0, 34, 85, 27, 6.0),
    ArmAction(   -90,  0, 12, 65, 27, 6.0),#中间
    ArmAction(   0,  0, 0, 0, 27, 6.0),
]

arm_sequence_reset = [
    ArmAction(   90,  65, -27, -90, -45, 12),
]

class ArmActionExecutor:
    def __init__(self):
        """初始化机械臂动作执行器"""
        rospy.init_node('arm_action_executor', anonymous=True)
        
        # 机械臂基本参数
        self.arm_serial_device = rospy.get_param("M2_serial", "/dev/ttyUSB2")
        self.arm_speed = rospy.get_param("M2_speed", 70)
        self.arm_acc = rospy.get_param("M2_acc", 20)
        self.delay_time = rospy.get_param("arm_delay_time", 2.0)
        
        # 度数控制模式
        self.degree_control = 122
        
        # 初始位置
        self.init_pose = [0.0, 0.0, 0.0, 0.0, 27.0]
        self.current_pose = list(self.init_pose)
        
        # 序列执行相关变量
        self.current_action_index = 0
        self.action_start_time = None
        self.current_sequence = None
        self.sequence_running = False
        
        # 串口连接
        try:
            self.serial = serial.Serial(
                port=self.arm_serial_device,
                baudrate=115200,
                timeout=1,
                write_timeout=1
            )
            rospy.loginfo("机械臂串口连接成功: %s", self.arm_serial_device)
        except Exception as e:
            rospy.logerr("机械臂串口连接失败: %s", str(e))
            raise
        
        # 订阅动作命令 - 兼容两种话题
        self.cmd_sub = rospy.Subscriber('/arm_action_cmd', Int32, self.action_callback, queue_size=10)
        self.sequence_sub = rospy.Subscriber('/arm_sequence_cmd', Int32, self.sequence_callback, queue_size=10)
        self.base_adjust_sub = rospy.Subscriber('/arm_base_adjust', Int32, self.base_adjust_callback, queue_size=10)
        
        # 发布机械臂完成状态
        self.completion_pub = rospy.Publisher('/arm_action_completed', Bool, queue_size=10)
        self.status_pub = rospy.Publisher('/arm_status', String, queue_size=10)
        
        rospy.loginfo("机械臂动作执行器初始化完成")
        
        # 初始化到起始位置
        self.move_to_init_position()
    
    def send_command(self, pose, speed=None, acc=None):
        """发送机械臂控制命令"""
        if speed is None:
            speed = self.arm_speed
        if acc is None:
            acc = self.arm_acc
            
        # 确保pose有5个元素 [base, shoulder, elbow, wrist, gripper]
        pose_to_send = list(pose)
        if len(pose_to_send) == 4:
            pose_to_send.append(0.0)  # 添加夹爪角度，默认为0
        
        command = {
            "T": self.degree_control,
            "b": float(pose_to_send[0]),  # base joint
            "s": float(pose_to_send[1]),  # shoulder joint  
            "e": float(pose_to_send[2]),  # elbow joint
            "w": float(pose_to_send[3]),  # wrist joint
            "h": float(pose_to_send[4]) if len(pose_to_send) > 4 else 0.0,  # gripper
            "spd": int(speed),
            "acc": int(acc)
        }
        
        try:
            command_str = json.dumps(command, ensure_ascii=False) + "\n"
            self.serial.write(command_str.encode('utf-8'))
            self.current_pose = pose_to_send[:5]
            rospy.loginfo("发送命令: %s", command)
        except Exception as e:
            rospy.logerr("发送命令失败: %s", str(e))
            raise
    
    def time_delay(self, seconds):
        """延时函数"""
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time < seconds) and not rospy.is_shutdown():
            rospy.sleep(0.01)
    
    def move_to_init_position(self):
        """移动到初始位置"""
        rospy.loginfo("机械臂移动到初始位置")
        self.send_command(self.init_pose)
        self.time_delay(3.0)

    def adjust_init_pose_base(self, delta):
        """按当前角度微调第一个舵机，并同步到 init_pose"""
        target_pose = list(self.current_pose)
        if len(target_pose) < 5:
            target_pose = list(self.init_pose)

        target_pose[0] += float(delta)
        self.init_pose[0] = target_pose[0]

        rospy.loginfo("第一个舵机角度微调 %+d，当前目标角度: %.1f", delta, target_pose[0])
        self.send_command(target_pose)

    def base_adjust_callback(self, msg):
        """处理来自 dipan.py 的第一个舵机微调命令"""
        if msg.data == 0:
            return
        self.adjust_init_pose_base(msg.data)
    
    def execute_action_sequence(self, action_list, delay_between_actions=None):
        """执行动作序列"""
        if delay_between_actions is None:
            delay_between_actions = self.delay_time
            
        rospy.loginfo("开始执行动作序列，共%d个动作", len(action_list))
        
        for i, pose in enumerate(action_list):
            if rospy.is_shutdown():
                break
                
            rospy.loginfo("执行第%d个动作: %s", i+1, pose)
            self.send_command(pose)
            self.time_delay(delay_between_actions)
        
        rospy.loginfo("动作序列执行完成")
    
    def execute_arm_action_sequence(self, arm_action_list):
        """执行ArmAction类型的动作序列（带时间控制）"""
        rospy.loginfo("开始执行ArmAction序列，共%d个动作", len(arm_action_list))
        
        for i, action in enumerate(arm_action_list):
            if rospy.is_shutdown():
                break
                
            rospy.loginfo("执行第%d个动作: base=%d, shoulder=%d, elbow=%d, wrist=%d, claw=%d, 持续时间=%.1fs", 
                         i+1, action.base, action.shoulder, action.elbow, action.wrist, action.claw, action.duration)
            
            # 发送动作命令
            pose = [action.base, action.shoulder, action.elbow, action.wrist, action.claw]
            self.send_command(pose)
            
            # 等待指定时间
            self.time_delay(action.duration)
        
        rospy.loginfo("ArmAction序列执行完成")
        
        # 发布完成状态
        completion_msg = Bool()
        completion_msg.data = True
        self.completion_pub.publish(completion_msg)
        
        status_msg = String()
        status_msg.data = "completed"
        self.status_pub.publish(status_msg)
        
        rospy.loginfo("🤖 机械臂序列完成信号已发送")
    
    def run_arm_sequence(self, sequence):
        """运行arm.py风格的序列（逐步执行）"""
        if not self.sequence_running:
            self.current_sequence = sequence
            self.current_action_index = 0
            self.action_start_time = None
            self.sequence_running = True
            rospy.loginfo("开始运行序列，共%d个动作", len(sequence))
        
        if self.current_action_index >= len(sequence):
            self.current_action_index = 0
            self.sequence_running = False
            rospy.loginfo("序列执行完成")
            return True  # 所有动作完成

        now = time.time()
        if self.action_start_time is None:
            action = sequence[self.current_action_index]
            rospy.loginfo('执行动作 %d: base=%d, shoulder=%d, elbow=%d, wrist=%d, claw=%d', 
                         self.current_action_index + 1, action.base, action.shoulder, action.elbow, action.wrist, action.claw)
            pose = [action.base, action.shoulder, action.elbow, action.wrist, action.claw]
            self.send_command(pose)
            self.action_start_time = now
            return False

        if now - self.action_start_time >= sequence[self.current_action_index].duration:
            self.current_action_index += 1
            self.action_start_time = None
        return False  # 尚未完成
    
    def predefined_action_1(self):
        """预定义动作1：左侧检测和授粉"""
        action_sequence = [
            [86.0, -26.3, 59.8, -0.9, 0],      # 右侧准备位置
            [90.1, -27.9, -37.1, -0.18, 0],    # 右侧检测位置
            [88.8, -25.8, -37.1, -0.88, 45],   # 右侧授粉位置1
            [88.8, 0, -37, -0.26, 28.91],      # 右侧授粉位置2（夹爪张开）
            [6.7, -25.5, 55.5, -0.45, 45],     # 右侧授粉位置3
            [0, 0, 90, 180, 0]              # 返回初始位置
        ]
        
        rospy.loginfo("执行预定义动作1：左侧检测和授粉")
        self.execute_action_sequence(action_sequence)
    
    def predefined_action_2(self):
        """预定义动作2：右侧检测和授粉"""
        action_sequence = [
            [-86.0, -26.3, 59.8, -0.9, 0],      # 右侧准备位置
            [-90.1, -27.9, -37.1, -0.18, 0],    # 右侧检测位置
            [-88.8, -25.8, -37.1, -0.88, 45],   # 右侧授粉位置1
            [-88.8, 0, -37, -0.26, 28.91],      # 右侧授粉位置2（夹爪张开）
            [-6.7, -25.5, 55.5, -0.45, 45],     # 右侧授粉位置3
            [0, 0, 90, 180, 0]                   # 返回初始位置
        ]
        
        rospy.loginfo("执行预定义动作2：右侧检测和授粉")
        self.execute_action_sequence(action_sequence)
    
    def predefined_action_3(self):
        """预定义动作3：完整的左右侧授粉循环"""
        action_sequence = [
            # 左侧动作
            [86.0, -26.3, 59.8, -0.9, 0],      # 右侧准备位置
            [90.1, -27.9, -37.1, -0.18, 0],    # 右侧检测位置
            [88.8, -25.8, -37.1, -0.88, 45],   # 右侧授粉位置1
            [88.8, 0, -37, -0.26, 28.91],      # 右侧授粉位置2（夹爪张开）
            [6.7, -25.5, 55.5, -0.45, 45],             # 左侧授粉位置2
            [0, 0, 90, 180, 0],                 # 中间位置
            
            # 右侧动作
            [-86.0, -26.3, 59.8, -0.9, 0],      # 右侧准备位置
            [-90.1, -27.9, -37.1, -0.18, 0],    # 右侧检测位置
            [-88.8, -25.8, -37.1, -0.88, 45],   # 右侧授粉位置1
            [-88.8, 0, -37, -0.26, 28.91],      # 右侧授粉位置2（夹爪张开）
            [-6.7, -25.5, 55.5, -0.45, 45],    # 右侧授粉位置3
            [0, 0, 90, 180, 0]                  # 返回初始位置
        ]
        
        rospy.loginfo("执行预定义动作3：完整左右侧授粉循环")
        self.execute_action_sequence(action_sequence, delay_between_actions=1.5)
    
    def predefined_action_4(self):
        """预定义动作4：简化检测动作"""
        action_sequence = [
            [0, -60, -60, 210],     # 准备位置
        ]
        
        rospy.loginfo("执行预定义动作4：简化检测动作")
        self.execute_action_sequence(action_sequence)
    
    def custom_action(self, pose_list):
        """执行自定义动作序列"""
        rospy.loginfo("执行自定义动作序列")
        self.execute_action_sequence(pose_list)
    
    def execute_action_by_id(self, action_id):
        """根据动作ID执行对应动作"""
        rospy.loginfo("🤖 开始执行机械臂动作ID: %d", action_id)
        
        # 发布开始状态
        status_msg = String()
        status_msg.data = "executing"
        self.status_pub.publish(status_msg)
        
        if action_id == 1:
            self.predefined_action_1()  # 执行动作1序列（左侧检测和授粉）
        elif action_id == 2:
            self.predefined_action_2()  # 执行动作2序列（右侧检测和授粉）
        elif action_id == 3:
            self.predefined_action_3()  # 执行动作3序列（完整左右侧授粉循环）
        elif action_id == 4:
            self.predefined_action_4()  # 执行动作4序列（简化检测动作）
        elif action_id == 666:
            self.execute_arm_action_sequence(arm_sequence_area1_0) # 测试舵机各角度
        elif action_id == 5:
            self.execute_arm_action_sequence(arm_sequence_area1_1)  # 执行area1_1序列
        elif action_id == 6:
            self.execute_arm_action_sequence(arm_sequence_area1_2)  # 执行area1_2序列
        elif action_id == 7:
            self.execute_arm_action_sequence(arm_sequence_area1_3)  # 执行area1_3序列
        elif action_id == 8:
            self.execute_arm_action_sequence(arm_sequence_area1_4)  # 执行area1_4序列
        elif action_id == 9:
            self.execute_arm_action_sequence(arm_sequence_reset)   # 执行复位序列
        elif action_id == 0:
            self.move_to_init_position()  # 移动到初始位置
            # 发布完成状态
            self._publish_completion_status()
        elif action_id == 99:
            # 关闭扭矩锁
            self.serial.write((json.dumps({"T": 210, "cmd": 0}) + '\n').encode())
            rospy.loginfo("扭矩锁已关闭")
            self._publish_completion_status()
        elif action_id == 100:
            # 开启扭矩锁
            self.serial.write((json.dumps({"T": 210, "cmd": 1}) + '\n').encode())
            rospy.loginfo("扭矩锁已开启")
            self._publish_completion_status()
        else:
            rospy.logwarn("未知的动作ID: %d", action_id)
            self._publish_completion_status()
    
    def _publish_completion_status(self):
        """发布机械臂完成状态"""
        completion_msg = Bool()
        completion_msg.data = True
        self.completion_pub.publish(completion_msg)
        
        status_msg = String()
        status_msg.data = "completed"
        self.status_pub.publish(status_msg)
        
        rospy.loginfo("🤖 机械臂动作完成信号已发送")
    
    def action_callback(self, msg):
        """动作命令回调函数"""
        action_id = msg.data
        rospy.loginfo("收到动作命令: %d", action_id)
        self.execute_action_by_id(action_id)
    
    def sequence_callback(self, msg):
        """序列命令回调函数（逐步执行模式）"""
        sequence_id = msg.data
        rospy.loginfo("收到序列命令: %d", sequence_id)
        
        if sequence_id == 1:
            sequence = arm_sequence_area1_1
        elif sequence_id == 2:
            sequence = arm_sequence_area1_2
        elif sequence_id == 3:
            sequence = arm_sequence_area1_3
        elif sequence_id == 4:
            sequence = arm_sequence_area1_4
        elif sequence_id == 0:
            self.move_to_init_position()
            return
        elif sequence_id == 99:
            # 关闭扭矩锁
            self.serial.write((json.dumps({"T": 210, "cmd": 0}) + '\n').encode())
            rospy.loginfo("扭矩锁已关闭")
            return
        elif sequence_id == 100:
            # 开启扭矩锁
            self.serial.write((json.dumps({"T": 210, "cmd": 1}) + '\n').encode())
            rospy.loginfo("扭矩锁已开启")
            return
        else:
            rospy.logwarn("未知的序列ID: %d", sequence_id)
            return
        
        # 执行序列（逐步执行模式）
        rate = rospy.Rate(10)  # 10Hz
        self.current_action_index = 0
        self.action_start_time = None
        self.sequence_running = True
        
        while not rospy.is_shutdown() and not self.run_arm_sequence(sequence):
            rate.sleep()
        
        rospy.loginfo("序列 %d 执行完成", sequence_id)
    
    def shutdown(self):
        """关闭串口连接"""
        # 关闭串口连接
        if hasattr(self, 'serial') and self.serial.is_open:
            try:
                self.serial.close()
                rospy.loginfo("机械臂串口连接已关闭")
            except Exception as e:
                rospy.logerr("关闭串口时发生错误: %s", str(e))

if __name__ == '__main__':
    try:
        # 创建机械臂执行器
        executor = ArmActionExecutor()
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr("程序发生错误: %s", str(e))
        import traceback
        rospy.logerr("错误详情: %s", traceback.format_exc())
    finally:
        if 'executor' in locals():
            executor.shutdown() 
