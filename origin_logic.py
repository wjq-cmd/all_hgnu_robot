#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from math import pi, sqrt
from std_msgs.msg import Int32, Int32MultiArray, String
from std_srvs.srv import Empty
import os
import random
import json
import socket
import tf.transformations
import subprocess
import threading
import time
try:
    import queue
except ImportError:
    import Queue as queue
from nav_msgs.msg import Odometry

try:
    from md5_report import RoundBroadcast, write_round_report, write_roundbook_report
except Exception as _md5_import_error:
    RoundBroadcast = None
    write_round_report = None
    write_roundbook_report = None

# 裁判软件 TCP 配置（端口以比赛现场公布为准）
REFEREE_HOST = '192.168.12.87'
REFEREE_PORT = 8888
ROBOT_ID = '1'
REPORT_INTERVAL = 0.5
RECONNECT_DELAY = 0.5

# ---------------------- 播报 MD5 报表配置 ----------------------
TEAM_NAME = '1'                         # 队伍号，写进报表
# 报表目录：和 origin_code.py 在同一个文件夹（scripts/）下的 MD5_Reports
MD5_REPORT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'MD5_Reports')
ARRIVAL_PAUSE = 1.0                     # 抵达化验区后停车保持的时间(秒)
DISTRACTOR_VOICE_PREFIX = 'distractor'  # 干扰样本语音：distractor1.wav ~ distractor4.wav

def log_text(text, level='info'):
    """安全打印中文：Python2 下转成 UTF-8 字节，避免 ascii 解码崩溃"""
    try:
        if not isinstance(text, str):
            text = text.encode('utf-8')
        if level == 'warn':
            rospy.logwarn(text)
        elif level == 'err':
            rospy.logerr(text)
        else:
            rospy.loginfo(text)
    except Exception:
        pass


TASK_MAP = {
    0: 'C', 1: 'A', 2: 'B',
    3: '4', 4: '3', 5: '2', 6: '1',
    7: 'R', 8: 'R', 9: 'R', 10: 'R',
}

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.init_waypoints()
        self.round_index = 0            # 已跑完的圈数
        self.round_records = []         # 已完成圈的播报记录（Ctrl+C 时统一导出）
        self._report_written = False    # 防止重复生成报表
        # 所有语音走同一个队列，按顺序播放，避免和 A/B/C 区播报重叠
        self._voice_queue = queue.Queue()
        self._voice_busy = False
        self._voice_cv = threading.Condition()
        self._voice_worker = threading.Thread(target=self._voice_worker_loop)
        self._voice_worker.daemon = True
        self._voice_worker.start()
        self.init_state()
        self.init_ros()
        self.voice_map = self.init_voice_map()
        self.voice_path = '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/'
        self.run_fsm()

    def init_waypoints(self):
        # 预计算四元数，避免重复计算
        euler_angles = (pi/2, pi/2, pi/2, -pi/2, -pi/2, -pi/2, -pi/2, 0, -pi, 0, 0)
        quaternions = [Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle)) 
                      for angle in euler_angles]
        
        self.waypoints = [
            Pose(Point(1.340,1.840, 0), quaternions[0]),      # C
            Pose(Point(0.680,2.210, 0), quaternions[1]),      # A
            Pose(Point(1.436,2.733, 0), quaternions[2]),      # B
            Pose(Point(-0.985, 1.111, 0), quaternions[3]),     # 4
            Pose(Point(-1.800, 1.580, 0), quaternions[4]),     # 3
            Pose(Point(0.985, 1.711, 0), quaternions[5]),     # 2
            Pose(Point(-0.600,2.000, 0), quaternions[6]),     # 1
            Pose(Point(-0.029,-0.172, 0), quaternions[7]),    # Start，起点
            Pose(Point(-0.335,3.625,0), quaternions[8]),     # Answer area 识别汉字的位置
            Pose(Point(0.513,0.080, 0), quaternions[9]),     # Recognition area388，识别二维码的位置
            Pose(Point(0.670,0.050, 0), quaternions[10])     # First time recognition area633，识别二维码的位置
            #   Pose(Point(0.666,-0.108, 0), quaternions[10])      # First time recognition area
            ]

    def init_state(self):
        self.state = 9  # 状态机阶段
        self.windows_C = 0
        self.windows_A = 0
        self.windows_B = 0
        self.windows_count = 0
        self.windows_1234 = 0
        self.qr_data_received = False
        self.ram_result = [0,0,0,0,0]
        self.is_first_run = True  # 标记是否是第一次启动
        self.qr_start_time = None  # QR识别开始时间
        self.recognition_ok = False  # 本圈二维码是否真正识别成功（随机兜底为 False）
        self.distractor_area = 0  # 干扰样本所在化验区(1~4)，0表示没识别到
        # 本圈播报记录（每圈结束时核验并生成 Excel）
        self.broadcast = RoundBroadcast() if RoundBroadcast is not None else None
        self.broadcast_archived = False
        # 裁判软件上报数据
        self.current_odom = [0.0, 0.0]
        self.current_speed = 0.0
        self.current_task = 'R'
        self.cv1_result = 'WAIT-0'
        self.cv2_result = ''
        self._report_lock = threading.Lock()
        self._referee_sock = None
        self._referee_sock_lock = threading.Lock()

    def init_ros(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)  # 减少队列大小
        self.reset_pub = rospy.Publisher('/reset_qr_data', Int32MultiArray, queue_size=5)
        self.start_qr_pub = rospy.Publisher('/start_qr_processing', Int32MultiArray, queue_size=5)
        self.cam_sub = rospy.Subscriber('/cam_return', Int32MultiArray, self.detect_result, queue_size=5)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=5)
        self.cv1_sub = rospy.Subscriber('/cv1_result', String, self.cv1_callback, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("Navigation initialized successfully")
        self.start_referee_report()

    def odom_callback(self, msg):
        with self._report_lock:
            self.current_odom = [
                round(msg.pose.pose.position.x, 2),
                round(msg.pose.pose.position.y, 2)
            ]
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.current_speed = round(sqrt(vx * vx + vy * vy), 2)

    def cv1_callback(self, msg):
        with self._report_lock:
            self.cv1_result = msg.data

    def set_task(self, task):
        with self._report_lock:
            self.current_task = task

    def task_from_waypoint(self, idx):
        return TASK_MAP.get(idx, 'R')

    def build_cv2(self):
        letters = []
        if self.windows_C:
            letters.append('C')
        if self.windows_A:
            letters.append('A')
        if self.windows_B:
            letters.append('B')
        if not letters or not self.windows_1234:
            return ''
        return ''.join(letters) + '-' + str(self.windows_1234)

    def update_cv2(self):
        with self._report_lock:
            self.cv2_result = self.build_cv2()

    def get_report_data(self):
        with self._report_lock:
            return {
                'id': ROBOT_ID,
                'speed': self.current_speed,
                'odom': list(self.current_odom),
                'task': self.current_task,
                'CV1': self.cv1_result,
                'CV2': self.cv2_result
            }

    def _ensure_referee_connection(self):
        """确保 TCP 长连接可用，断线自动重连。"""
        with self._referee_sock_lock:
            if self._referee_sock is not None:
                return True
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((REFEREE_HOST, REFEREE_PORT))
                self._referee_sock = sock
                rospy.loginfo('Connected to referee %s:%d' % (REFEREE_HOST, REFEREE_PORT))
                return True
            except Exception as e:
                rospy.logwarn('Failed to connect to referee (%s:%d): %s' % (
                    REFEREE_HOST, REFEREE_PORT, str(e)))
                return False

    def send_to_referee(self, data):
        payload = (json.dumps(data, separators=(',', ': ')) + '\n').encode('utf-8')
        with self._referee_sock_lock:
            sock = self._referee_sock
            if sock is None:
                rospy.logwarn('Referee not connected, dropping report')
                return False
            try:
                sock.sendall(payload)
                rospy.logdebug('Sent to referee: %s' % payload.decode('utf-8').rstrip())
                return True
            except Exception as e:
                rospy.logwarn('Send to referee failed (%s:%d): %s' % (
                    REFEREE_HOST, REFEREE_PORT, str(e)))
                try:
                    sock.close()
                except Exception:
                    pass
                self._referee_sock = None
                return False

    def referee_report_loop(self):
        rate = rospy.Rate(1.0 / REPORT_INTERVAL)
        while not rospy.is_shutdown():
            if self._ensure_referee_connection():
                self.send_to_referee(self.get_report_data())
            else:
                rospy.sleep(RECONNECT_DELAY)
            rate.sleep()

    def start_referee_report(self):
        t = threading.Thread(target=self.referee_report_loop)
        t.daemon = True
        t.start()
        rospy.loginfo('Referee report started: %s:%d' % (REFEREE_HOST, REFEREE_PORT))

    def init_voice_map(self):
        return {
            ('xue', 1): 'xue1.wav', ('xue', 2): 'xue2.wav', ('xue', 3): 'xue3.wav',
            ('tiye', 1): 'tiye1.wav', ('tiye', 2): 'tiye2.wav', ('tiye', 3): 'tiye3.wav',
            ('mianyi', 1): 'mianyi1.wav', ('mianyi', 2): 'mianyi2.wav', ('mianyi', 3): 'mianyi3.wav',
            ('jisu', 1): 'jisu1.wav', ('jisu', 2): 'jisu2.wav', ('jisu', 3): 'jisu3.wav',
            ('C', 1): 'C1.wav', ('C', 2): 'C2.wav', ('C', 3): 'C3.wav', ('C', 4): 'C4.wav',
            ('A', 1): 'A1.wav', ('A', 2): 'A2.wav', ('A', 3): 'A3.wav', ('A', 4): 'A4.wav',
            ('A', 5): 'A5.wav', ('A', 6): 'A6.wav', ('A', 7): 'A7.wav', ('A', 8): 'A8.wav',
            ('B', 1): 'B1.wav', ('B', 2): 'B2.wav', ('B', 3): 'B3.wav', ('B', 4): 'B4.wav',
            ('B', 5): 'B5.wav', ('B', 6): 'B6.wav', ('B', 7): 'B7.wav', ('B', 8): 'B8.wav',
            ('B', 9): 'B9.wav', ('B', 10): 'B10.wav', ('B', 11): 'B11.wav', ('B', 12): 'B12.wav',
            ('B', 13): 'B13.wav', ('B', 14): 'B14.wav', ('B', 15): 'B15.wav', ('B', 16): 'B16.wav',
        }

    def run_fsm(self):
        rate = rospy.Rate(10)  # 添加10Hz的频率控制，避免空转占用CPU
        while not rospy.is_shutdown():
            if self.state == 9:  # 起点到识别区
                # 根据是否第一次启动选择不同的识别区位置
                recognition_waypoint = 10 if self.is_first_run else 9
                if self.goto_waypoint(recognition_waypoint):
                    rospy.loginfo("到达识别区 (第一次启动: {})".format(self.is_first_run))
                    self.is_first_run = False  # 标记第一次启动已完成
                    self.state = 10
                    self.start_qr_process()
                    self.wait_for_qr_result()
                    self.announce_recognition()
            elif self.state == 10:  # 识别区到配药区
                self.handle_medicine_area()
                self.state = 12
            elif self.state == 12:  # 配药区到答题区
                if self.goto_waypoint(8):
                    rospy.loginfo("到达答题区")
                    self.state = 13
                    rospy.sleep(1)
            elif self.state == 13:  # 答题区到数字区
                target_index = 7 - self.windows_1234
                if self.goto_waypoint(target_index):
                    rospy.loginfo("到达数字区")
                    self.announce_arrival()
                    self.finish_round()  # 停车并保持1秒 -> 缓存本圈记录
                    self.reset_state()
            else:
                # 未知状态时休眠，避免CPU空转
                rate.sleep()
            
            rate.sleep()  # 控制循环频率

    def goto_waypoint(self, idx):
        self.set_task('R')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.waypoints[idx]
        if self.move(goal):
            self.set_task(self.task_from_waypoint(idx))
            return True
        return False

    def start_qr_process(self):
        # 使用单一消息对象，避免重复创建
        reset_msg = Int32MultiArray()
        reset_msg.data = [0]
        self.reset_pub.publish(reset_msg)
        rospy.sleep(0.5)
        start_msg = Int32MultiArray()
        start_msg.data = [1]
        self.start_qr_pub.publish(start_msg)
        rospy.loginfo("Started QR code processing")
        self.qr_data_received = False

    def generate_random_result(self):
        """生成随机的识别结果"""
        # 随机选择要去的数字窗口 (1-4)
        self.windows_1234 = random.randint(1, 4)
        
        # 随机选择要去的字母窗口数量 (1-3)
        num_windows = random.randint(1, 3)
        
        # 重置所有窗口标志
        self.windows_C = 0
        self.windows_A = 0
        self.windows_B = 0
        
        # 随机选择要去的窗口
        windows = ['C', 'A', 'B']
        selected_windows = random.sample(windows, num_windows)
        
        for window in selected_windows:
            if window == 'C':
                self.windows_C = 1 #
            elif window == 'A':
                self.windows_A = 1
            elif window == 'B':
                self.windows_B = 1
        
        self.windows_count = num_windows
        self.recognition_ok = False  # 随机兜底，不是真识别
        self.qr_data_received = True
        self.update_cv2()

        rospy.loginfo("随机生成识别结果: C:{}, A:{}, B:{}, count:{}, window:{}".format(
            self.windows_C, self.windows_A, self.windows_B, self.windows_count, self.windows_1234))

    def wait_for_qr_result(self):
        self.qr_start_time = rospy.Time.now()
        timeout = rospy.Duration(12)
        rate = rospy.Rate(10)  # 10Hz检查，识别完成后立即播报
        while not rospy.is_shutdown():
            if self.qr_data_received:
                rospy.loginfo("收到二维码识别结果")
                break
            if (rospy.Time.now() - self.qr_start_time) > timeout:
                rospy.logwarn("Timeout waiting for QR result, using random result")
                self.generate_random_result()
                break
            rate.sleep()

    def handle_medicine_area(self):
        rospy.loginfo("Moving to medicine area")
        
        # 预计算窗口配置，避免重复计算
        window_configs = [
            (0, self.windows_C, 'C'),
            (1, self.windows_A, 'A'), 
            (2, self.windows_B, 'B')
        ]
        
        for idx, flag, label in window_configs:
            if flag == 1:
                if self.goto_waypoint(idx):
                    rospy.loginfo("取到{}窗口样本".format(label))
                    # 异步语音播放，不阻塞导航
                    self.play_medicine_area_voice(label)
                    rospy.sleep(0.5)  # 适当减少等待时间，因为语音播放是异步的
        rospy.loginfo("Completed medicine area visits, proceeding to answer area")

    def play_medicine_area_voice(self, label):
        """异步播放药品区域的语音"""
        voice_key = self.get_voice_key(label)
        if voice_key:
            filename = self.voice_map.get(voice_key)
            if filename:
                self._play_voice_file(filename, self.describe_sample_voice(label), u'取样')
            else:
                rospy.logwarn("未找到语音文件: {}".format(voice_key))

    def get_voice_key(self, label):
        """获取语音文件的键值 - 处理药品区域的语音播报"""
        if label == 'C' and not (self.windows_A == 1 or self.windows_B == 1):
            return ('C', self.windows_1234)
        elif label == 'A':
            if not (self.windows_B == 1 or self.windows_C == 1):
                return ('A', self.windows_1234)
            elif not self.windows_B == 1 and self.windows_C == 1:
                return ('A', 4 + self.windows_1234)
        elif label == 'B':
            if not (self.windows_A == 1 or self.windows_C == 1):
                return ('B', self.windows_1234)
            elif not self.windows_A == 1 and self.windows_C == 1:
                return ('B', 4 + self.windows_1234)
            elif self.windows_A == 1 and self.windows_C == 1:
                return ('B', 8 + self.windows_1234)
            elif self.windows_A == 1 and not self.windows_C == 1:
                return ('B', 12 + self.windows_1234)
        return None

    def get_number_area_voice_key(self):
        """获取数字区域的语音文件键值"""
        # 根据样本类型和窗口数量确定语音
        sample_types = {1: 'xue', 2: 'tiye', 3: 'mianyi', 4: 'jisu'}
        sample_type = sample_types.get(self.windows_1234)
        if sample_type:
            return (sample_type, self.windows_count)
        return None

    def sample_names_text(self):
        """本轮要取的样本类别，如 A、C"""
        names = []
        if self.windows_A:
            names.append('A')
        if self.windows_B:
            names.append('B')
        if self.windows_C:
            names.append('C')
        return u'、'.join(names) if names else u'未知'

    def describe_sample_voice(self, label):
        """取样播报内容（裁判软件 MD5 窗口里填这一行）"""
        return u'取样播报：在%s区取样，本轮取%s样本，送往第%d化验区' % (
            label, self.sample_names_text(), self.windows_1234)

    def describe_number_voice(self):
        """送样播报内容"""
        sample_cn = {1: u'血液', 2: u'体液', 3: u'免疫', 4: u'激素'}.get(self.windows_1234, u'未知')
        return u'送样播报：%s样本%d份，送往第%d化验区' % (
            sample_cn, self.windows_count, self.windows_1234)

    def announce_recognition(self):
        try:
            self._announce_recognition()
        except Exception as e:
            log_text(u'识别播报异常: %s' % str(e), 'warn')

    def _announce_recognition(self):
        if not self.recognition_ok:
            # 识别失败 → 随机生成任务，不播报干扰样本，直接继续正常流程
            rospy.loginfo("识别未成功，使用随机任务，不播报干扰样本")
            return
        if not self.distractor_area:
            rospy.loginfo("本轮没有识别到干扰样本")
            return
        text = u'干扰样本在第%d化验区' % self.distractor_area
        filename = '%s%d.wav' % (DISTRACTOR_VOICE_PREFIX, self.distractor_area)
        self._play_voice_file(filename, text, u'识别')

    def _voice_worker_loop(self):
        """唯一的语音播放线程：所有播报排队，保证不重叠、不互相打断"""
        while not rospy.is_shutdown():
            try:
                kind, payload = self._voice_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            with self._voice_cv:
                self._voice_busy = True
            try:
                if kind == 'file':
                    os.system('play ' + payload + ' 2>/dev/null')
                else:
                    safe = payload.replace('"', '').replace("'", '')
                    os.system('espeak -v zh "%s" 2>/dev/null' % safe)
                log_text(u'语音播放完成: %s' % payload)
            except Exception as e:
                rospy.logwarn("Voice playback failed: %s" % str(e))
            finally:
                with self._voice_cv:
                    self._voice_busy = False
                    self._voice_cv.notify_all()
                self._voice_queue.task_done()

    def _espeak(self, text):
        """没有对应语音文件时用 espeak 兜底（同样走语音队列）"""
        if text:
            self._voice_queue.put(('espeak', text))

    def wait_voice_done(self, timeout=20.0):
        """等语音队列全部播完（最多等 timeout 秒）"""
        deadline = time.time() + timeout
        with self._voice_cv:
            while time.time() < deadline:
                if not self._voice_busy and self._voice_queue.unfinished_tasks == 0:
                    return True
                self._voice_cv.wait(min(0.2, max(0.01, deadline - time.time())))
        rospy.logwarn("等待语音播放超时")
        return False

    def cancel_navigation(self):
        """只在导航还在进行时取消，避免对已完成目标发 cancel 触发 PREEMPTING 警告"""
        try:
            state = self.move_base.get_state()
        except Exception:
            state = None
        if state in (0, 1):  # PENDING / ACTIVE
            try:
                self.move_base.cancel_goal()
            except Exception:
                pass

    def stop_robot(self, hold_seconds=None):
        """抵达化验区：取消导航，并持续发零速度在原地保持指定时间"""
        if hold_seconds is None:
            hold_seconds = ARRIVAL_PAUSE
        log_text(u'抵达化验区，停车保持 %.1f 秒' % hold_seconds)
        self.cancel_navigation()
        deadline = time.time() + hold_seconds
        while time.time() < deadline:
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.1)

    def finish_round(self):
        """抵达化验区：停车保持 ARRIVAL_PAUSE 秒，然后缓存本圈记录"""
        self.stop_robot(ARRIVAL_PAUSE)
        self.archive_current_round()

    def archive_current_round(self):
        """把本圈播报存进内存，等 Ctrl+C 时统一写 Excel"""
        if self.broadcast is None or self.broadcast_archived:
            return
        ok = self.broadcast.verify()
        self.round_index += 1
        self.round_records.append({
            'index': self.round_index,
            'rows': [dict(row) for row in self.broadcast.rows],
            'verified': ok,
            'fingerprint': self.broadcast.fingerprint(),
        })
        self.broadcast_archived = True
        log_text(u'第%d圈播报%d条已缓存（Ctrl+C 时生成Excel）' % (
            self.round_index, len(self.broadcast.rows)))

    def write_final_report(self):
        """Ctrl+C / ROS关闭时：把已完成圈和当前半圈写成一个Excel"""
        if self._report_written:
            return None
        self._report_written = True
        if write_roundbook_report is None:
            rospy.logerr("MD5报表不可用：md5_report.py 没有正确加载")
            return None

        rounds = [dict(item) for item in self.round_records]
        current = self.broadcast
        if current is not None and current.rows and not self.broadcast_archived:
            ok = current.verify()
            rounds.append({
                'index': self.round_index + 1,
                'rows': [dict(row) for row in current.rows],
                'verified': ok,
                'fingerprint': current.fingerprint(),
            })
        meta = {'team': TEAM_NAME, 'time': time.strftime('%Y-%m-%d %H:%M:%S')}
        paths = write_roundbook_report(MD5_REPORT_DIR, rounds, meta)
        if paths:
            log_text(u'已生成MD5报表：%s' % paths['xlsx'])
        else:
            rospy.logerr("MD5报表生成失败，目录：%s" % MD5_REPORT_DIR)
        return paths

    def _play_voice_file(self, filename, content=None, category='播报'):
        """记录播报内容（用于MD5），并把语音放进队列按顺序播放"""
        path = self.voice_path + filename
        exists = os.path.isfile(path)
        if content and self.broadcast is not None:
            try:
                self.broadcast.add(category, content, filename,
                                   u'' if exists else u'语音文件缺失，使用espeak兜底')
            except Exception as e:
                log_text(u'记录播报异常: %s' % str(e), 'warn')
        if exists:
            self._voice_queue.put(('file', path))
            log_text(u'语音入队: %s | %s' % (filename, content))
        else:
            rospy.logwarn("语音文件不存在: %s，使用espeak兜底" % path)
            self._espeak(content)

    def move(self, goal):
        self.move_base.send_goal(goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return False
        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            return True
        return False

    def update_distractor_area(self, data):
        """origin_code 发来的第6个数：干扰样本所在化验区(1~4)，0 表示没有"""
        area = 0
        if len(data) >= 6:
            try:
                area = int(data[5])
            except Exception:
                area = 0
        if area < 0 or area > 4:
            area = 0
        if area != self.distractor_area:
            self.distractor_area = area
            if area:
                rospy.loginfo("识别到干扰样本：第%d化验区" % area)

    def detect_result(self, msg):
        # 添加状态检查，避免不必要的处理
        if self.state != 10 or self.qr_data_received:
            return
            
        # 直接赋值，减少列表操作
        data = msg.data
        if len(data) >= 5:  # 安全检查
            # 第6个数是干扰样本所在化验区(origin_code 新加的)
            self.update_distractor_area(data)
            # 检查是否是重新识别的特殊消息
            if data[0] == -1 and data[1] == -1 and data[2] == -1 and data[3] == -1 and data[4] == -1:
                rospy.loginfo("收到空框消息，重置超时时间，继续等待重新识别...")
                # 重置超时时间，继续等待识别结果
                self.qr_start_time = rospy.Time.now()
                return
            
            self.windows_C = data[0]
            self.windows_A = data[1]
            self.windows_B = data[2]
            self.windows_count = data[3]
            self.windows_1234 = data[4]
            self.recognition_ok = True  # 真正识别到二维码
            self.qr_data_received = True
            self.update_cv2()
            rospy.loginfo("识别结果: C:{}, A:{}, B:{}, count:{}, window:{}".format(
                self.windows_C, self.windows_A, self.windows_B, self.windows_count, self.windows_1234))

    def reset_state(self):
        # 保存第一次启动标志，避免重置时丢失
        first_run_flag = self.is_first_run
        self.init_state()
        self.is_first_run = first_run_flag  # 恢复第一次启动标志
        rospy.loginfo("状态已重置，等待新一轮任务")

    def play_number_area_voice(self):
        """异步播放数字区域的语音"""
        voice_key = self.get_number_area_voice_key()
        if voice_key:
            filename = self.voice_map.get(voice_key)
            if filename:
                self._play_voice_file(filename, self.describe_number_voice(), u'送样')
            else:
                rospy.logwarn("未找到语音文件: {}".format(voice_key))

    def announce_arrival(self):
        """到达数字区域后播报"""
        rospy.loginfo("Reached window %d" % self.windows_1234)
        self.play_number_area_voice()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Ctrl+C：先把本圈/已完成圈的播报写成Excel，再停车
        try:
            self.write_final_report()
        except Exception as e:
            log_text(u'生成MD5报表异常: %s' % str(e), 'err')
        self.cancel_navigation()
        rospy.sleep(2)
        # 发布空的Twist消息停止机器人
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        with self._referee_sock_lock:
            if self._referee_sock:
                try:
                    self._referee_sock.close()
                except Exception:
                    pass
                self._referee_sock = None

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
