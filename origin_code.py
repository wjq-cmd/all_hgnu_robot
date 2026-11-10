#!/usr/bin/python
# -*- coding:utf8 -*-

import os
import time
import cv2
import rospy
import math
import numpy as np
from std_msgs.msg import Int32MultiArray

approxPolyDP_epslion=0.02#多边形近似参数，越小越精准
wh_rate=0.4#长宽比系数，越小越接近正方形
min_area=500
max_area=20000

min_center_distance=20#中心距离

# 初始化微信二维码识别器
wechat_detector = cv2.wechat_qrcode_WeChatQRCode(
    "detect.prototxt",
    "detect.caffemodel",
    "sr.prototxt",
    "sr.caffemodel"
)
std_detector = cv2.QRCodeDetector()

def decode_qr_crop(crop):
    """优先用微信模型；解不出时用 OpenCV 标准识别器补一次（干扰码 ACD 用得上）"""
    try:
        data, _ = wechat_detector.detectAndDecode(crop)
        if len(data) > 0 and data[0]:
            return data[0]
    except Exception as e:
        rospy.logwarn("[QR] 微信模型识别异常: %s" % e)
    try:
        text = std_detector.detectAndDecode(crop)[0]
        if text:
            rospy.loginfo("[QR] 标准识别器补上: %s" % text)
        return text
    except Exception as e:
        rospy.logwarn("[QR] 标准识别器识别异常: %s" % e)
        return ''


def distance(point1, point2):
    """计算距离"""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def filter_qr_data(data):
    """识别到的二维码内容只要包含字符D，就忽略该二维码（当作未识别）"""
    if 'D' in data.upper():
        return '0'
    return data

def find_distractor_area(datas):
    """干扰样本(D)在第几个化验区，返回1~4；没有返回0"""
    for i in range(len(datas)):
        if 'D' in datas[i].upper():
            return i + 1
    return 0

rospy.init_node('detect_abc', anonymous=True)

pub_flag = rospy.Publisher('/cam_return', Int32MultiArray, queue_size=10)

cap = None  # 摄像头对象，按需打开关闭
should_process = False  # 是否进行识别
wait_until_time = 0  # 等待重新开始的时间戳

# 识别超时时间（秒）
QR_TIMEOUT = 4

# 识别控制回调
def start_qr_processing_cb(msg):
    global cap, should_process, qr_start_time, wait_until_time
    # 如果正在等待期间，也响应新的启动请求（允许立即重新开始）
    if wait_until_time > 0 and time.time() < wait_until_time:
        rospy.loginfo("[QR] 等待期间收到启动请求，立即重新开始识别...")
    
    wait_until_time = 0  # 重置等待时间
    if cap is not None:
        cap.release()
        cap = None
    cap = cv2.VideoCapture("http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw")
    should_process = True
    qr_start_time = time.time()
    rospy.loginfo("[QR] 摄像头已打开，开始识别...")

def reset_qr_data_cb(msg):
    global cap, should_process, wait_until_time
    should_process = False
    wait_until_time = 0  # 重置等待时间
    if cap is not None:
        cap.release()
        cap = None
        rospy.loginfo("[QR] 摄像头已关闭，重置识别状态")

# 订阅识别控制话题
rospy.Subscriber('/start_qr_processing', Int32MultiArray, start_qr_processing_cb)
rospy.Subscriber('/reset_qr_data', Int32MultiArray, reset_qr_data_cb)

qr_start_time = 0

while not rospy.is_shutdown():
    # 检查是否在等待期间
    if wait_until_time > 0:
        if time.time() < wait_until_time:
            time.sleep(0.1)
            continue
        else:
            # 等待结束，重新开始识别
            wait_until_time = 0
            cap = cv2.VideoCapture("http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw")
            should_process = True
            qr_start_time = time.time()
            rospy.loginfo("[QR] 等待结束，重新开始识别...")
    
    if not should_process or cap is None:
        time.sleep(0.1)  # 优化sleep时间从0.05改为0.1
        continue
    # 超时自动关闭
    if time.time() - qr_start_time > QR_TIMEOUT:
        should_process = False
        if cap is not None:
            cap.release()
            cap = None
            rospy.loginfo("[QR] 识别超时，摄像头已关闭")
        continue
    # 开始用摄像头读数据，返回hx为true则表示读成功，frame为读的图像
    hx, frame = cap.read()
    if not hx:
        rospy.logwarn('[QR] 读取摄像头失败')
        continue
    # 获取图像的高度和宽度
    height, width = frame.shape[:2]

    # 定义旋转角度（正值表示逆时针旋转）
    angle = 5

    # 计算旋转中心
    center = (width // 2, height // 2)

    # 定义旋转矩阵
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)

    # 执行旋转操作
    frame = cv2.warpAffine(frame, matrix, (width, height))

    grayImg=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(grayImg, 50, 150, apertureSize=3)
    
    # 兼容OpenCV 3.x和4.x版本的findContours
    contours_result = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_result) == 3:
        # OpenCV 3.x返回 (image, contours, hierarchy)
        _, contours, _ = contours_result
    else:
        # OpenCV 4.x返回 (contours, hierarchy)
        contours, _ = contours_result

    #筛选条件0 面积
    contours2=[]
    for i in range(len(contours)):
        M1 = cv2.moments(contours[i])
        if M1['m00']<min_area or M1['m00']>max_area:
            continue
        contours2.append(contours[i])

    #筛选条件1 四边形
    sibianxing=[]
    for c in contours2:
        # 对轮廓进行多边形逼近
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, approxPolyDP_epslion * peri, True)
        hull=approx
        if len(hull)==4:
            sibianxing.append(hull)

    #筛选条件2 正方形
    #判断正方形方案1，判断长宽比
    zhengfangxing=[]
    for hull in sibianxing:
        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect
        abs(w - h)
        if abs(w - h)/(w+h)<wh_rate:#边长归一标准差
            zhengfangxing.append(hull)

    #筛选条件3 正方形的包含关系
    dingweikuang=[]
    jieguo=[]
    for i in range(len(zhengfangxing)-1):
        M1 = cv2.moments(zhengfangxing[i])
        if M1['m00']==0:
             continue
        cx1 = int(M1['m10'] / M1['m00'])
        cy1= int(M1['m01'] / M1['m00'])
        for j in range(i+1,len(zhengfangxing)):
            M2 = cv2.moments(zhengfangxing[j])
            if M2['m00']==0:#两者面积为0则跳过
                continue
            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])
            
            #1二者中心互相包含
            res1=cv2.pointPolygonTest(zhengfangxing[i],(cx2,cy2), False)
            res2=cv2.pointPolygonTest(zhengfangxing[j],(cx1,cy1), False)
            if res1>0 and res2>0:#如果两个轮廓中心具有包含关系         
                #2计算中心距离
                distance_=distance((cx1,cy1),(cx2,cy2))
                if distance_<min_center_distance:#中心距里小于5个像素      
                    dingweikuang+=[zhengfangxing[i],zhengfangxing[j]]
                    jieguo.append(zhengfangxing[i])
    
    # 只有检测到足够的定位框才进行处理
    if(len(dingweikuang)==48):
        min_x = 1000
        max_x = 0
        min_y = 1000
        max_y = 0
        for i in range(len(jieguo)):
            for j in range(len(jieguo[0])):
                if jieguo[i][j][0][0] < min_x:
                    min_x = jieguo[i][j][0][0]
                    min_x_index = [i,j]
                if jieguo[i][j][0][0] > max_x:
                    max_x = jieguo[i][j][0][0]
                    max_x_index = [i,j]
                if jieguo[i][j][0][1] < min_y:
                    min_y = jieguo[i][j][0][1]
                    min_y_index = [i,j]
                if jieguo[i][j][0][1] > max_y:
                    max_y = jieguo[i][j][0][1]
                    max_y_index = [i,j]

        x0 = jieguo[min_x_index[0]][min_x_index[1]][0][0]
        y0 = jieguo[min_x_index[0]][min_x_index[1]][0][1]
        x1 = jieguo[max_x_index[0]][max_x_index[1]][0][0]
        y1 = jieguo[max_x_index[0]][max_x_index[1]][0][1]
        x2 = jieguo[min_y_index[0]][min_y_index[1]][0][0]
        y2 = jieguo[min_y_index[0]][min_y_index[1]][0][1]
        x3 = jieguo[max_y_index[0]][max_y_index[1]][0][0]
        y3 = jieguo[max_y_index[0]][max_y_index[1]][0][1]
        
        real_box_centers = [(x0,y0),(x1,y1),(x2,y2),(x3,y3)]
        if min_x_index[0] < max_x_index[0]:
            goal_box_centers=[(50,550),(550,50),(50,50),(550,550)]
        else:
            goal_box_centers=[(50,50),(550,550),(550,50),(50,550)]

        dst_pts = np.array(goal_box_centers, dtype=np.float32)
        src_pts = np.array(real_box_centers, dtype=np.float32)
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped_img = cv2.warpPerspective(frame, M, (600,600))
        
        # 提取四个区域进行QR码识别
        aera1_crop = warped_img[65:255, 65:235]
        aera2_crop = warped_img[65:255, 365:535]
        aera3_crop = warped_img[335:525, 65:235]
        aera4_crop = warped_img[335:525, 365:535]

        # 优先微信模型，解不出时标准识别器兜底（干扰码 ACD 用得上）
        img0_data = decode_qr_crop(aera1_crop) or '0'
        img1_data = decode_qr_crop(aera2_crop) or '0'
        img2_data = decode_qr_crop(aera3_crop) or '0'
        img3_data = decode_qr_crop(aera4_crop) or '0'

        # 干扰样本内容含D，先记下它在哪个化验区，再从取样逻辑里剔除
        distractor_area = find_distractor_area([img0_data, img1_data, img2_data, img3_data])
        if distractor_area:
            rospy.loginfo("[QR] 干扰样本在第%d化验区" % distractor_area)
        # 识别内容包含字符D则忽略该二维码
        img0_data = filter_qr_data(img0_data)
        img1_data = filter_qr_data(img1_data)
        img2_data = filter_qr_data(img2_data)
        img3_data = filter_qr_data(img3_data)

        # 检查是否所有QR码都没有识别到
        if img0_data == '0' and img1_data == '0' and img2_data == '0' and img3_data == '0':
            rospy.loginfo("[QR] 检测到四个框但没有识别到二维码，通知主控程序需要重新识别...")
            # 发送特殊消息通知主控程序需要重新识别
            msg = Int32MultiArray()
            msg.data = [-1, -1, -1, -1, -1, distractor_area]  # 最后一位：干扰样本所在化验区
            pub_flag.publish(msg)
            
            # 关闭摄像头
            should_process = False
            if cap is not None:
                cap.release()
                cap = None
            # 设置等待时间为10秒后
            wait_until_time = time.time() + 10
            continue

        #字符串长度比较
        img_len =[0,0,0,0] 
        img_len[0] = 0 if img0_data == '0' else len(img0_data)
        img_len[1] = 0 if img1_data == '0' else len(img1_data)
        img_len[2] = 0 if img2_data == '0' else len(img2_data)
        img_len[3] = 0 if img3_data == '0' else len(img3_data)

        max_img = max(img_len)
        windows_count = img_len.index(max_img) + 1
        max_message = [img0_data,img1_data,img2_data,img3_data]
        
        msg = Int32MultiArray()
        msg.data = [1,0,1,2,1]

        if max_img == 3:
            msg.data = [1,1,1,3,windows_count]#0.是否去c，1.是否去a，2.是否去b，3.样品数 4.self.windows_1234  
        elif max_img == 2:
            # 根据具体识别内容判断AB、BC、AC组合
            if max_message[windows_count - 1] == 'AB':
                msg.data = [0,1,1,2,windows_count]  # AB
            elif max_message[windows_count - 1] == 'BC':
                msg.data = [1,0,1,2,windows_count]  # BC
            elif max_message[windows_count - 1] == 'AC':
                msg.data = [1,1,0,2,windows_count]  # AC
        elif max_img == 1:
            if max_message[windows_count - 1] == 'A':
                msg.data = [0,1,0,1,windows_count]
            elif max_message[windows_count - 1] == 'B':
                msg.data = [0,0,1,1,windows_count]
            elif max_message[windows_count - 1] == 'C':
                msg.data = [1,0,0,1,windows_count]

        msg.data.append(distractor_area)  # 第6个数：干扰样本所在化验区(0=没有)
        pub_flag.publish(msg)#发布数据

        #打印结果
        rospy.loginfo("win1 %s win2 %s win3 %s win4 %s goto %d .",img0_data,img1_data,img2_data,img3_data,windows_count)
        
        # 识别完成后关闭摄像头
        should_process = False
        if cap is not None:
            cap.release()
            cap = None
            rospy.loginfo("[QR] 识别完成，摄像头已关闭")
    
    # 监测键盘输入是否为q，为q则退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):       # 按q退出
        break

# 释放摄像头
if cap is not None:
    cap.release()

# 结束所有窗口
cv2.destroyAllWindows() 





