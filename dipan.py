#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
底盘自动运动控制系统 - 前进、后退、左转、右转
自动执行运动序列，并与机械臂集成
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String, Bool
from enum import Enum
import threading
import time
import os
import glob
import sys

try:
    import msvcrt
except ImportError:
    msvcrt = None
    import select
    import termios
    import tty

class MotionState(Enum):
    """运动状态枚举"""
    IDLE = "idle"
    INITIALIZING = "initializing"
    MOVING_FORWARD = "moving_forward"
    MOVING_BACKWARD = "moving_backward"
    TURNING_LEFT = "turning_left"
    TURNING_RIGHT = "turning_right"
    WAITING_FOR_ARM = "waiting_for_arm"
    EXECUTING_ARM_ACTION = "executing_arm_action"
    TASK_COMPLETED = "task_completed"
    PAUSED = "paused"
    STOPPED = "stopped"

class MotionStep:
    """运动步骤类"""
    def __init__(self, motion_type, duration, arm_action_id=None, wait_time=0.5):
        """
        motion_type: 'forward', 'backward', 'left', 'right', 'stop'
        duration: 运动持续时间（秒）
        arm_action_id: 机械臂动作ID（None表示不执行机械臂动作）
        wait_time: 运动完成后的等待时间（秒）
        """
        self.motion_type = motion_type
        self.duration = duration
        self.arm_action_id = arm_action_id
        self.wait_time = wait_time

class SimpleMotionSystem:
    """自动运动控制系统 - 使用原始速度控制，支持自动任务序列"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('auto_motion_system', anonymous=True)
        
        # 系统配置
        self.config = {
            'linear_speed': 0.21,      # 线速度 m/s
            'angular_speed': 3.0,     # 角速度 rad/s
            'arm_action_timeout': 100.0,  # 机械臂动作超时时间（秒）
        }
        
        # 状态变量
        self.motion_state = MotionState.IDLE
        self.state_start_time = None
        self.motion_duration = 0.0  # 当前运动持续时间
        self.arm_action_start_time = None
        self.arm_action_completed = False  # 机械臂完成标志
        self.wait_start_time = None  # 等待开始时间
        
        # 任务序列相关
        self.task_sequence = []  # 任务序列
        self.current_step_index = 0  # 当前步骤索引
        self.auto_running = False  # 是否正在自动运行
        
        # 定义任务序列（示例：根据之前的导航点转换）
        self._define_task_sequence()
        
        # 速度发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # ROS通信
        self.state_pub = rospy.Publisher('/motion_state', String, queue_size=10)
        self.progress_pub = rospy.Publisher('/task_progress', String, queue_size=10)
        self.control_sub = rospy.Subscriber('/motion_control', String, self.control_callback, queue_size=10)
        
        # 机械臂控制发布者
        self.arm_action_pub = rospy.Publisher('/arm_action_cmd', Int32, queue_size=10)
        self.arm_action_pub = rospy.Publisher('/arm_action_cmd', Int32, queue_size=10)
        self.arm_adjust_pub = rospy.Publisher('/arm_base_adjust', Int32, queue_size=10)
        
        # 订阅机械臂完成状态
        self.arm_completion_sub = rospy.Subscriber('/arm_action_completed', Bool, self.arm_completion_callback, queue_size=10)
        self.arm_status_sub = rospy.Subscriber('/arm_status', String, self.arm_status_callback, queue_size=10)
        
        # 播放启动音频（播放完毕后才继续）
        self._play_startup_audio()

        # 启动后监听键盘 1/2，微调第一个舵机角度
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop)
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        # 启动主循环
        self.main_thread = threading.Thread(target=self._main_loop)
        self.main_thread = threading.Thread(target=self._main_loop)
        self.main_thread.daemon = True
        self.main_thread.start()
        
    
    def _play_startup_audio(self):
        """播放同目录下的MP3启动音频，播放完毕后才返回"""
        import pygame

        script_dir = os.path.dirname(os.path.abspath(__file__))
        mp3_files = glob.glob(os.path.join(script_dir, "*.mp3"))

        if not mp3_files:
            rospy.logwarn("[AUTO_MOTION] 未找到MP3音频文件，跳过播放")
            return

        audio_file = mp3_files[0]
        rospy.loginfo("[AUTO_MOTION] 🔊 正在播放启动音频: %s", os.path.basename(audio_file))

        try:
            pygame.mixer.init()
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            rospy.loginfo("[AUTO_MOTION] 启动音频播放完毕")
        except Exception as e:
            rospy.logerr("[AUTO_MOTION] 播放音频失败: %s", str(e))

    def _define_task_sequence(self):
        """定义任务序列 - 这里可以根据实际需求修改"""
        # 示例任务序列：前进、机械臂动作、转向、前进...
        self.task_sequence = [
            # 第一排作业
             MotionStep('forward', 2.8, arm_action_id=6, wait_time=0.5),   #A1 
             MotionStep('forward', 2.3, arm_action_id=7, wait_time=0.5),   # A2
             MotionStep('forward', 2.4, arm_action_id=8, wait_time=0.5),   # A3
            
            # 转向第二排
            MotionStep('backward', 8.0, arm_action_id=None, wait_time=0.5),   # 前进到点5，执行动作5
            MotionStep('forward',5.0, arm_action_id=None, wait_time=0.5),  # 右转90度
            MotionStep('right', 0.8, arm_action_id=None, wait_time=0.5),   # 前进到点6，执行动作6
            
            # # 第二排作业
            MotionStep('forward', 2.0, arm_action_id=None, wait_time=0.5),  # 后退到点7，执行动作7
            MotionStep('forward', 2.0, arm_action_id=None, wait_time=0.5),  # 后退到点8，执行动作8
            MotionStep('forward', 2.0, arm_action_id=None, wait_time=0.5),  # 后退到点9，执行动作5
            MotionStep('forward', 5.0, arm_action_id=None, wait_time=0.5),  # 后退到点10，执行动作6
            MotionStep('forward', 7.5, arm_action_id=None, wait_time=0.5),

            # # 转向第三排
            # MotionStep('left', 0.9, arm_action_id=None, wait_time=0.5),  # 右转90度
            # MotionStep('forward', 4.3, arm_action_id=None, wait_time=0.5),   # 前进到点666，执行动作7
            
            # # 第三排作业
            # MotionStep('left', 0.8, arm_action_id=None, wait_time=0.5),   # 左转调整
            # MotionStep('forward', 2.5, arm_action_id=666, wait_time=0.5),   # B1
            # MotionStep('forward', 2.3, arm_action_id=666, wait_time=0.5),   # B2
            # MotionStep('forward', 2.1, arm_action_id=666, wait_time=0.5),   # B3
            # MotionStep('forward', 2.1, arm_action_id=666, wait_time=0.5),   # B4

            # # 转向第四排
            # MotionStep('forward', 2.5, arm_action_id=None, wait_time=0.5), # 前进调整位置
            # MotionStep('right', 0.9, arm_action_id=None, wait_time=0.5),  # 右转
            # MotionStep('forward', 3.5, arm_action_id=None, wait_time=0.5),   # 前进到点15，执行动作7
            # MotionStep('right', 0.8, arm_action_id=None, wait_time=0.5),  #右转

            # # 第四排作业
            # MotionStep('forward', 2.0, arm_action_id=666, wait_time=0.5),  # C1
            # MotionStep('forward', 2.3, arm_action_id=666, wait_time=0.5),  # C2
            # MotionStep('forward', 2.3, arm_action_id=666, wait_time=0.5),  # C3
            # MotionStep('forward', 2.3, arm_action_id=666, wait_time=0.5),  # C4

            # # 返回起点
            # MotionStep('forward', 1.9, arm_action_id=None, wait_time=0.5), # 后退调整
            # MotionStep('left', 0.9, arm_action_id=None, wait_time=0.5),   # 左转
            # MotionStep('forward', 4.5, arm_action_id=None, wait_time=0.5), # 前进
            # MotionStep('left', 0.8, arm_action_id=None, wait_time=0.5),   # 左转
            # MotionStep('forward', 12.0, arm_action_id=None, wait_time=0.5), # 前进
            # MotionStep('left', 1.6, arm_action_id=None, wait_time=0.5),   # 左转调整
            # MotionStep('forward', 10.0, arm_action_id=9, wait_time=1.0),  # 前进到终点，执行复位动作9
        ]
    
    def _change_state(self, new_state):
        """改变运动状态"""
        if new_state != self.motion_state:
            rospy.loginfo("[AUTO_MOTION] 状态切换: %s -> %s", 
                         self.motion_state.value, new_state.value)
            self.motion_state = new_state
            self.state_start_time = time.time()
            
            # 发布状态
            msg = String()
            msg.data = new_state.value
            self.state_pub.publish(msg)
            
            # 发布进度
            self._publish_progress()
    
    def _publish_progress(self):
        """发布任务进度"""
        if len(self.task_sequence) > 0:
            progress = (self.current_step_index / float(len(self.task_sequence))) * 100.0
            progress_msg = String()
            progress_msg.data = "进度: {}/{} ({:.1f}%)".format(
                self.current_step_index, len(self.task_sequence), progress)
            self.progress_pub.publish(progress_msg)

    def _keyboard_loop(self):
        """监听键盘 1/2 并发布机械臂基座微调命令"""
        rospy.loginfo("[AUTO_MOTION] 键盘微调已启用：按 1 基座角度 -5，按 2 基座角度 +5")

        if msvcrt is not None:
            while not rospy.is_shutdown():
                try:
                    if msvcrt.kbhit():
                        self._handle_keyboard_key(msvcrt.getwch())
                    time.sleep(0.1)
                except Exception as e:
                    rospy.logwarn("[AUTO_MOTION] 键盘监听异常: %s", str(e))
                    time.sleep(0.5)
            return

        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
        except Exception as e:
            rospy.logwarn("[AUTO_MOTION] 当前终端不支持键盘监听: %s", str(e))
            return

        try:
            tty.setcbreak(fd)
            while not rospy.is_shutdown():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable:
                    self._handle_keyboard_key(sys.stdin.read(1))
        except Exception as e:
            rospy.logwarn("[AUTO_MOTION] 键盘监听异常: %s", str(e))
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _handle_keyboard_key(self, key):
        """处理 1/2 键的机械臂基座微调"""
        if key == '1':
            self._publish_arm_base_adjust(-5)
        elif key == '2':
            self._publish_arm_base_adjust(5)

    def _publish_arm_base_adjust(self, delta):
        """发布机械臂第一个舵机角度微调命令"""
        msg = Int32()
        msg.data = delta
        self.arm_adjust_pub.publish(msg)
        rospy.loginfo("[AUTO_MOTION] 已发送基座微调命令: %+d", delta)

    def _send_zero_velocity(self):
        """发布零速度命令（不改变状态）"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _main_loop(self):
        """主控制循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            try:
                current_time = time.time()

                # 处理运动状态
                self._handle_motion_state(current_time)

                # 非移动状态时持续发送零速度，确保底盘保持静止
                if self.motion_state not in [
                    MotionState.MOVING_FORWARD,
                    MotionState.MOVING_BACKWARD,
                    MotionState.TURNING_LEFT,
                    MotionState.TURNING_RIGHT
                ]:
                    self._send_zero_velocity()

                rate.sleep()

            except Exception as e:
                rospy.logerr("[AUTO_MOTION] 主循环错误: %s", str(e))
                self._stop_motion()
    
    def _handle_motion_state(self, current_time):
        """处理运动状态"""
        # 空闲状态
        if self.motion_state == MotionState.IDLE:
            pass
        
        # 初始化状态
        elif self.motion_state == MotionState.INITIALIZING:
            if current_time - self.state_start_time > 2.0:
                rospy.loginfo("[AUTO_MOTION] 初始化完成，开始执行任务")
                self.current_step_index = 0
                self._execute_current_step()
        
        # 运动状态（前进、后退、左转、右转）
        elif self.motion_state in [MotionState.MOVING_FORWARD, MotionState.MOVING_BACKWARD, 
                                    MotionState.TURNING_LEFT, MotionState.TURNING_RIGHT]:
            if self.motion_duration > 0 and (current_time - self.state_start_time) >= self.motion_duration:
                rospy.loginfo("[AUTO_MOTION] ⏱️ 运动完成")
                self._stop_motion()
                # 进入等待状态
                if self.auto_running:
                    self.wait_start_time = current_time
                    self._change_state(MotionState.WAITING_FOR_ARM)
        
        # 等待执行机械臂状态
        elif self.motion_state == MotionState.WAITING_FOR_ARM:
            if self.current_step_index < len(self.task_sequence):
                step = self.task_sequence[self.current_step_index]
                if current_time - self.wait_start_time >= step.wait_time:
                    # 检查是否需要执行机械臂动作
                    if step.arm_action_id is not None:
                        self._execute_arm_action(step.arm_action_id)
                    else:
                        # 没有机械臂动作，直接进入下一步
                        self._move_to_next_step()
        
        # 执行机械臂动作状态
        elif self.motion_state == MotionState.EXECUTING_ARM_ACTION:
            self._handle_arm_action_state(current_time)
        
        # 任务完成状态
        elif self.motion_state == MotionState.TASK_COMPLETED:
            if current_time - self.state_start_time > 5.0:
                rospy.loginfo("[AUTO_MOTION] 🎉 所有任务已完成！")
                self.state_start_time = current_time
        
        # 暂停状态
        elif self.motion_state == MotionState.PAUSED:
            pass
    
    def _handle_arm_action_state(self, current_time):
        """处理机械臂动作状态"""
        # 检查是否收到机械臂完成信号
        if self.arm_action_completed:
            rospy.loginfo("[AUTO_MOTION] ✅ 机械臂动作完成")
            self.arm_action_completed = False  # 重置标志
            # 如果是自动运行模式，进入下一步
            if self.auto_running:
                self._move_to_next_step()
            else:
                self._change_state(MotionState.IDLE)
            return
        
        # 检查机械臂动作是否超时
        if self.arm_action_start_time is not None:
            elapsed_time = current_time - self.arm_action_start_time
            if elapsed_time >= self.config['arm_action_timeout']:
                rospy.logwarn("[AUTO_MOTION] ⚠️ 机械臂动作超时（%.1f秒）", elapsed_time)
                self.arm_action_completed = False
                # 如果是自动运行模式，继续下一步
                if self.auto_running:
                    self._move_to_next_step()
                else:
                    self._change_state(MotionState.IDLE)
                return
    
    def _execute_current_step(self):
        """执行当前步骤"""
        if self.current_step_index >= len(self.task_sequence):
            rospy.loginfo("[AUTO_MOTION] 🎉 所有任务步骤已完成！")
            self._change_state(MotionState.TASK_COMPLETED)
            self.auto_running = False
            return
        
        step = self.task_sequence[self.current_step_index]
        rospy.loginfo("[AUTO_MOTION] 📍 执行步骤 %d/%d: %s %.1f秒%s", 
                     self.current_step_index + 1, len(self.task_sequence),
                     step.motion_type, step.duration,
                     f", 机械臂动作={step.arm_action_id}" if step.arm_action_id else "")
        
        # 执行运动
        if step.motion_type == 'forward':
            self._move_forward(step.duration)
        elif step.motion_type == 'backward':
            self._move_backward(step.duration)
        elif step.motion_type == 'left':
            self._turn_left(step.duration)
        elif step.motion_type == 'right':
            self._turn_right(step.duration)
        elif step.motion_type == 'stop':
            self._stop_motion()
            if self.auto_running:
                self.wait_start_time = time.time()
                self._change_state(MotionState.WAITING_FOR_ARM)
    
    def _move_to_next_step(self):
        """移动到下一步"""
        self.current_step_index += 1
        self._execute_current_step()
    
    def _stop_motion(self):
        """停止所有运动"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        if not self.auto_running:
            self._change_state(MotionState.STOPPED)
    
    def _move_forward(self, duration=0.0):
        """前进"""
        rospy.loginfo("[AUTO_MOTION] ⬆️ 前进 (%.1f秒)", duration if duration > 0 else float('inf'))
        self.motion_duration = duration
        self._change_state(MotionState.MOVING_FORWARD)
        
        twist = Twist()
        twist.linear.x = self.config['linear_speed']
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def _move_backward(self, duration=0.0):
        """后退"""
        rospy.loginfo("[AUTO_MOTION] ⬇️ 后退 (%.1f秒)", duration if duration > 0 else float('inf'))
        self.motion_duration = duration
        self._change_state(MotionState.MOVING_BACKWARD)
        
        twist = Twist()
        twist.linear.x = -self.config['linear_speed']
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def _turn_left(self, duration=0.0):
        """左转"""
        rospy.loginfo("[AUTO_MOTION] ⬅️ 左转 (%.1f秒)", duration if duration > 0 else float('inf'))
        self.motion_duration = duration
        self._change_state(MotionState.TURNING_LEFT)
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.config['angular_speed']
        self.cmd_vel_pub.publish(twist)
    
    def _turn_right(self, duration=0.0):
        """右转"""
        rospy.loginfo("[AUTO_MOTION] ➡️ 右转 (%.1f秒)", duration if duration > 0 else float('inf'))
        self.motion_duration = duration
        self._change_state(MotionState.TURNING_RIGHT)
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.config['angular_speed']
        self.cmd_vel_pub.publish(twist)
    
    def control_callback(self, msg):
        """控制命令回调"""
        command = msg.data.strip().lower()
        rospy.loginfo("[AUTO_MOTION] 🎮 收到控制命令: %s", command)
        
        # 解析命令格式: "forward 2.0" 表示前进2秒
        parts = command.split()
        action = parts[0]
        
        # 自动任务控制命令
        if action == "start":
            if self.motion_state == MotionState.IDLE:
                rospy.loginfo("[AUTO_MOTION] 🏁 开始自动任务")
                self.auto_running = True
                self._change_state(MotionState.INITIALIZING)
            else:
                rospy.logwarn("[AUTO_MOTION] 当前状态不允许开始: %s", self.motion_state.value)
        
        elif action == "pause":
            if self.auto_running:
                rospy.loginfo("[AUTO_MOTION] ⏸️ 暂停自动任务")
                self.auto_running = False
                self._stop_motion()
                self._change_state(MotionState.PAUSED)
            else:
                rospy.logwarn("[AUTO_MOTION] 当前没有运行自动任务")
        
        elif action == "resume":
            if self.motion_state == MotionState.PAUSED:
                rospy.loginfo("[AUTO_MOTION] ▶️ 恢复自动任务")
                self.auto_running = True
                self._execute_current_step()
            else:
                rospy.logwarn("[AUTO_MOTION] 当前状态不允许恢复: %s", self.motion_state.value)
        
        elif action == "stop":
            rospy.loginfo("[AUTO_MOTION] 🛑 停止任务")
            self.auto_running = False
            self._stop_motion()
            self._change_state(MotionState.IDLE)
        
        elif action == "reset":
            rospy.loginfo("[AUTO_MOTION] 🔄 重置任务")
            self.auto_running = False
            self._stop_motion()
            self.current_step_index = 0
            self._change_state(MotionState.IDLE)
        
        # 手动控制命令（只在非自动模式下生效）
        elif not self.auto_running:
            duration = float(parts[1]) if len(parts) > 1 else 0.0
            
            if action == "forward" or action == "f":
                self._move_forward(duration)
            elif action == "backward" or action == "b":
                self._move_backward(duration)
            elif action == "left" or action == "l":
                self._turn_left(duration)
            elif action == "right" or action == "r":
                self._turn_right(duration)
            elif action == "arm":
                # 执行机械臂动作: "arm 5" 表示执行动作5
                if len(parts) > 1:
                    arm_action_id = int(parts[1])
                    self._execute_arm_action(arm_action_id)
                else:
                    rospy.logwarn("[AUTO_MOTION] 机械臂命令需要指定动作ID: arm <id>")
            elif action == "status":
                self._print_status()
            elif action == "setspeed":
                # 设置速度: "setspeed 0.3 0.6" 表示设置线速度0.3, 角速度0.6
                if len(parts) >= 3:
                    self.config['linear_speed'] = float(parts[1])
                    self.config['angular_speed'] = float(parts[2])
                    rospy.loginfo("[AUTO_MOTION] ⚙️ 速度已更新 - 线速度: %.2f m/s, 角速度: %.2f rad/s",
                                 self.config['linear_speed'], self.config['angular_speed'])
                else:
                    rospy.logwarn("[AUTO_MOTION] 速度设置格式: setspeed <线速度> <角速度>")
            else:
                rospy.logwarn("[AUTO_MOTION] 未知命令: %s", command)
        else:
            rospy.logwarn("[AUTO_MOTION] 自动模式运行中，手动命令被忽略: %s", command)
    
    def _execute_arm_action(self, arm_action_id):
        """执行机械臂动作"""
        rospy.loginfo("[AUTO_MOTION] 🤖 执行机械臂动作: %d", arm_action_id)
        
        # 发布机械臂动作命令
        arm_action_msg = Int32()
        arm_action_msg.data = arm_action_id
        self.arm_action_pub.publish(arm_action_msg)
        
        # 记录动作开始时间，重置完成标志
        self.arm_action_start_time = time.time()
        self.arm_action_completed = False
        self._change_state(MotionState.EXECUTING_ARM_ACTION)
    
    def _print_status(self):
        """打印当前状态"""
        if self.current_step_index < len(self.task_sequence):
            step = self.task_sequence[self.current_step_index]
            rospy.loginfo("  当前动作: %s %.1f秒%s", step.motion_type, step.duration,
                         f", 机械臂={step.arm_action_id}" if step.arm_action_id else "")
        progress = (self.current_step_index / float(len(self.task_sequence))) * 100.0 if len(self.task_sequence) > 0 else 0.0
    
    def arm_completion_callback(self, msg):
        """机械臂完成状态回调"""
        if msg.data:
            rospy.loginfo("[AUTO_MOTION] 🤖 收到机械臂完成信号")
            self.arm_action_completed = True
    
    def arm_status_callback(self, msg):
        """机械臂状态回调"""
        rospy.logdebug("[AUTO_MOTION] 机械臂状态: %s", msg.data)

if __name__ == '__main__':
    try:
        system = SimpleMotionSystem()
        system.auto_running = True
        
        # # 检查是否自动启动
        # auto_start = rospy.get_param("~auto_start", False)  # 默认不自动启动
        # start_delay = rospy.get_param("~start_delay", 3.0)  # 启动延时，默认3秒
        
        # if auto_start:
        #     rospy.loginfo("[AUTO_MOTION] 🚀 将在 %.1f 秒后自动启动任务...", start_delay)
        #     rospy.sleep(start_delay)
        #     rospy.loginfo("[AUTO_MOTION] ⏰ 启动延时到达，开始自动任务！")
        #     system.auto_running = True
        system._change_state(MotionState.INITIALIZING)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[AUTO_MOTION] 系统错误: %s", str(e)) 
