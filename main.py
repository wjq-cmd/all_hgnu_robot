import time
import cv2
import threading
import numpy as np
import rospy
import math
from functools import wraps
import sys

from image_Tag_converter import ImgConverter
from image_Tag_converter import TagConverter

sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")
from leju import base_action


#定义一些参数
Chest_img = None
ChestOrg = None

marker = None

chest_circle_x = None
chest_circle_y = None
Debug = 0

# 整体节奏：调大则更慢、更稳（只改这里即可）
PAUSE_STEP = 1.15        # 普通行走/转向/后退/小侧移
PAUSE_BOX_MOVE = 1.5     # 抱块侧移、抱块转向
PAUSE_BOX_WALK = 2.2     # 抱块向前走（负荷大，多停一会）
PAUSE_LOOP = 0.22        # 主循环与找箱控频

# 不同色块的hsv范围
color_range = {
    'green': [( 38 , 92 , 91 ),( 87 , 255 , 255 )],
    'orange': [( 10 , 150 , 120 ),( 30 , 255 , 255 )]
}
# color_range = {
#     # 绿色方块（适配你这张图，抗反光/阴影）
#     'green': [(35, 80, 80), (90, 255, 255)],
#     # 橙色方块（适配你这张图，抗干扰）
#     'orange': [(5, 130, 110), (35, 255, 255)]
# }


# ******************************************动作函数***********************************************a
#放下箱子
def Box_Down(n):   
    for i in range (0,n):
        base_action.action("PutCubeDown")
        time.sleep(PAUSE_STEP)

#后退
def Box_Back(n):
    for i in range (0,n):
            base_action.action("BoxBack1Run2")
            time.sleep(PAUSE_STEP)
def Box_Back1(n):
    for i in range (0,n):
            base_action.action("BoxBack2-1")
            time.sleep(PAUSE_STEP)
def Back2(n):
    for i in range (0,n):
            base_action.action("Back2Run")
            time.sleep(PAUSE_STEP)
def Back1(n):
    for i in range (0,n):
            base_action.action("Back1Run")
            time.sleep(PAUSE_STEP)

#前进#
def go_fast(n):   #快速前进一步
    for i in range (0,n):
        base_action.action("FastForward1s")
        time.sleep(PAUSE_STEP)
def go_fast1(n):   #快速前进一步
    for i in range (0,n):
        base_action.action("FastForward1s")
        time.sleep(PAUSE_STEP)
def go_fast2(n):   # 原两步合一，改为连续两次单步，减小惯性摔倒
    for i in range(0, n):
        base_action.action("FastForward2s")
        time.sleep(PAUSE_STEP)
def go_fast3(n):   # 原三步合一，改为连续三次单步
    for i in range(0, n):
        base_action.action("FastForward3s")
        time.sleep(PAUSE_STEP)
def box_go(n):
    for i in range (0,n):
        base_action.action("BoxForward3s_2")
        time.sleep(PAUSE_STEP)
def box_go1(n):    #抱着箱子前进一步
    for i in range (0,n):
        base_action.action("BoxForward3s_2")
        time.sleep(PAUSE_BOX_WALK)
def box_go2(n):     # 原抱块两步合一，改为两次「BoxForward2s_2」
    for i in range(0, n):
        base_action.action("BoxForward3s_2")
        time.sleep(PAUSE_BOX_WALK)
        base_action.action("BoxForward3s_2")
        time.sleep(PAUSE_BOX_WALK)
def box_go3(n):     # 原抱块三步合一，改为三次「BoxForward2s_2」
    for i in range(0, n):
        base_action.action("BoxForward3s_2")
        time.sleep(PAUSE_BOX_WALK)
        base_action.action("BoxForward3s_2")
        time.sleep(PAUSE_BOX_WALK)
        base_action.action("BoxForward3s_2")
        time.sleep(PAUSE_BOX_WALK)


#侧移#
def L_move1 (n):           #左侧移
    for i in  range (0,n):
        base_action.action ("Left02move")
        print('Left shift 1.5cm')
        time.sleep(PAUSE_STEP)
def L_move2 (n):           #左侧移
    for i in  range (0,n):
        base_action.action('move_left')
        time.sleep(PAUSE_STEP)
def BoxL_move1 (n):           #左侧移  
    for i in  range (0,n):
        base_action.action ("BoxLeft1")
        time.sleep(PAUSE_BOX_MOVE)
def BoxL_move2 (n):           #左侧移
    for i in  range (0,n):
        base_action.action('Box_move_left')
        time.sleep(PAUSE_BOX_MOVE)
def R_move1 (n):           #右侧移  
    for i in  range (0,n):
        base_action.action ("Right02move")
        time.sleep(PAUSE_STEP)
def R_move2 (n):           #右侧移  
    for i in  range (0,n):
        base_action.action('move_right')
        time.sleep(PAUSE_STEP)
def BoxR_move1 (n):           #右侧移 
    for i in  range (0,n):
        base_action.action ("抱块右侧移3")
        time.sleep(PAUSE_BOX_MOVE)
def BoxR_move2 (n):           #右侧移 
    for i in  range (0,n):
        base_action.action('抱块右侧移3')
        time.sleep(PAUSE_BOX_MOVE)

#转向#
def L_turn1 (n):           #左转  左转7°
    for i in  range (0,n):
        base_action.action("turn003L")
        time.sleep(PAUSE_STEP)
def L_turn2 (n):
    for i in range (0,n):
        base_action.action("LeftTurn1s")
        time.sleep(PAUSE_STEP)
def BoxL_turn1(n):           #左转 
    for i in  range (0,n):
        base_action.action("抱块小幅度左转4")
        time.sleep(PAUSE_BOX_MOVE)
def BoxL_turn2(n):           #左转 
    for i in  range (0,n):
        base_action.action("抱块小幅度左转4")
        time.sleep(PAUSE_BOX_MOVE)
def R_turn1 (n):           #右转 
    for i in  range (0,n):
        base_action.action("turn003R")
        time.sleep(PAUSE_STEP)
def R_turn2 (n):           #右转 
    for i in  range (0,n):
        base_action.action("RightTurn1s")
        time.sleep(PAUSE_STEP)
def BoxR_turn1 (n):           #右转
    for i in  range (0,n):
        base_action.action("抱块小幅度右转3")
        time.sleep(PAUSE_BOX_MOVE)
def BoxR_turn2 (n):           #右转
    for i in  range (0,n):
        base_action.action("BoxRightTurn1s")
        time.sleep(PAUSE_BOX_MOVE)


#获取图像
def get_img():
    global Chest_img,ChestOrg
    global ret
    image_reader_chest = ImgConverter()
    while True:
        ret, ChestOrg = image_reader_chest.chest_image()
        time.sleep(0.3)
        if ChestOrg is not None:
            Chest_img = ChestOrg
            time.sleep(0.05)
            #Chest_img = cv2.flip(Chest_img, 1)
        else:
            time.sleep(0.3)
            print("Temporarily unable to acquire image")
th2 = threading.Thread(target=get_img)
th2.setDaemon(True)
th2.start()

#查找方块
def find_box(img,color_name):
    global chest_circle_x, chest_circle_y
    if Chest_img is None:
        print('Waiting for image acquisition...')
        time.sleep(0.3)
    else:
        box_img = img
        box_img_bgr = cv2.cvtColor(box_img, cv2.COLOR_RGB2BGR)  # 将图片转换到BRG空间
        box_img_hsv = cv2.cvtColor(box_img, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        box_img = cv2.GaussianBlur(box_img_hsv, (3, 3), 0)  # 高斯模糊
        box_img_mask = cv2.inRange(box_img, color_range[color_name][0], color_range[color_name][1])  # 二值化
        box_img_closed = cv2.erode(box_img_mask, None, iterations=2)  # 腐蚀
        box_img_opened = cv2.dilate(box_img_mask, np.ones((4, 4), np.uint8), iterations=2)  # 膨胀    先腐蚀后运算等同于开运算
        (contours, hierarchy) = cv2.findContours(box_img_opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) != 0:
            area = []
            for cn in contours:
                contour_area = math.fabs(cv2.contourArea(cn))
                area.append(contour_area)
            max_index = np.argmax(area)
            (chest_circle_x,chest_circle_y),chest_radius = cv2.minEnclosingCircle(contours[max_index])
            cv2.circle(img,(int(chest_circle_x),int(chest_circle_y)),int(chest_radius),(0,0,255))
            print('A','x=',chest_circle_x,'y=',chest_circle_y)

            if Debug:
                # cv2.imwrite('image.png',img)
                cv2.imshow("Box", img)
                cv2.waitKey(2000)

        else:
            print('Searching for target...')

#搬箱子
def goto_box():
    global level, ID
    if chest_circle_x is None :
        print('等待中获取坐标中...')
        time.sleep(0.3)
    else:
        if chest_circle_x < 275:
            print("正在左侧移 ",chest_circle_x)
            base_action.action("Left3move")
            time.sleep(PAUSE_STEP)
        elif chest_circle_x < 295: 
            print("正在左侧移 ",chest_circle_x)
            base_action.action("Left02move")
            time.sleep(PAUSE_STEP)
        elif chest_circle_x > 365:    
            print("正在右侧移 ",chest_circle_x)
            base_action.action("Right3move")
            time.sleep(PAUSE_STEP)
        elif chest_circle_x > 345:    
            print("正在右侧移 ",chest_circle_x)
            base_action.action("Right02move")
            time.sleep(PAUSE_STEP)
        else:
            if chest_circle_y < 300:  
                print("前进",chest_circle_y)
                base_action.action("FastForward1s")
                time.sleep(PAUSE_STEP)
            else:
                print("开始抱箱子")
                base_action.action("FastForward1s")
                time.sleep(PAUSE_STEP)
                base_action.action("FastForward1s")
                time.sleep(PAUSE_STEP)
                base_action.action("GrabCube")
                base_action.action("LiftCubeUp")
                time.sleep(PAUSE_STEP)
                level = "end_box"


# ***************************************tag对正****************************************
def turn_to_tag(
    dis_x,
    dis_y,
    theta,
    x_offset=0,
    y_offset=0,
    theta_offset=0,
    x_threshold=0.05,
    y_threshold=0.03,
    theta_threshold=8,
):
    """对正 AR tag。x/y 阈值单位为米，theta 为度。
    三个 *_threshold 越大 → 容差越宽、越容易判「对正完成」；要更严则改小。"""
    is_turn_done = False
    # x_error=dis_x-x_offset：负表示比目标更近。原先用 x_error < x_threshold-0.03
    # 会在 x_error≈0 时仍成立（例如 0 < 0.07），导致一直误判「太近」而后退，永远对不齐。
    X_TOO_CLOSE = -0.025   # 比目标近超过约 2.5cm：后退；略近于此则仍可判完成
    X_DONE_MAX = x_threshold + 0.08   # 比目标远在此范围内且 y/θ 合格即认为完成

    x_error = dis_x-x_offset
    y_error = dis_y-y_offset
    theta_error = theta-theta_offset
    print("ID:", marker[0], "dis_x:", dis_x, "dis_y:", dis_y, "theta:", theta, "theta_offset", theta_offset)
    print("x_error:", x_error, "y_error:", y_error, "theta_error:", theta_error)

    # 已落在容差内则直接完成，避免 else 链与误触发后退打架
    if (
        X_TOO_CLOSE <= x_error <= X_DONE_MAX
        and abs(y_error) <= y_threshold
        and abs(theta_error) <= theta_threshold
    ):
        return True

    # 不偏离赛道
    if step == 1:
        if x_error < X_TOO_CLOSE:
            print("Moving backward (too close)", x_error, " < ", X_TOO_CLOSE)
            Box_Back(1)

        elif (theta_error+theta_threshold > 35):
            print("Left turn 1", theta_error, " > ", 14+theta_threshold)
            BoxL_turn2(1)
        elif (theta_error-theta_threshold < -35):
            print("Right turn 1", theta_error, " < ", -14-theta_threshold)
            BoxR_turn2(1)
        
        # 左右平移
        elif (y_error > y_threshold+0.05):
            print("Left move 1", y_error, " > ", y_threshold)
            BoxL_move2(1)
        elif (y_error < -y_threshold-0.05):
            print("Right move 1 ", y_error," < ", -y_threshold)
            BoxR_move2(1)

        # 快速靠近标签
        elif (x_error > x_threshold+0.2):
            print("Moving forward to approach tag", x_error, " > ", x_threshold+0.12)
            box_go(1)

        # 方向平行与标签
        elif (theta_error > theta_threshold):
            print("Left turn 2 ", theta_error, " > ", theta_threshold)
            BoxL_turn1(1)
        elif (theta_error < -theta_threshold):
            print("Right turn 2 ", theta_error, " < ", theta_threshold)
            BoxR_turn1(1)

        # 精确调整左右平移
        elif (y_error > y_threshold):
            print("Left move 1", y_error, " > ", y_threshold)
            BoxL_move1(1)
        elif (y_error < -y_threshold):
            print("Right move 1 ", y_error," < ", -y_threshold)
            BoxR_move1(1)

        # 靠近标签
        elif (x_error > x_threshold+0.12):
            print("Moving forward to approach tag", x_error, " > ", x_threshold+0.08)
            box_go(1)
        elif (x_error > x_threshold+0.06):
            print("Moving forward to approach tag", x_error, " > ", x_threshold+0.06)
            box_go(1)
        else:
            is_turn_done = True

         
    elif step == 2:
        if (theta_error+theta_threshold > 35):
            print("Left turn 1", theta_error, " > ", 14+theta_threshold)
            L_turn2(1)
        elif (theta_error-theta_threshold < -35):
            print("Right turn 1", theta_error, " < ", -14-theta_threshold)
            R_turn2(1)
        
        # 左右平移
        elif (y_error > y_threshold+0.05):
            print("Left move 1", y_error, " > ", y_threshold)
            L_move2(1)
        elif (y_error < -y_threshold-0.05):
            print("Right move 1 ", y_error," < ", -y_threshold)
            R_move2(1)

        # 快速靠近标签
        elif (x_error > x_threshold+0.2):
            print("Moving forward to approach tag", x_error, " > ", x_threshold+0.12)
            go_fast2(1)

        # 方向平行与标签
        elif (theta_error > theta_threshold):
            print("Left turn 2 ", theta_error, " > ", theta_threshold)
            L_turn1(1)
        elif (theta_error < -theta_threshold):
            print("Right turn 2 ", theta_error, " < ", theta_threshold)
            R_turn1(1)

        # 精确调整左右平移
        elif (y_error > y_threshold):
            print("Left move 1", y_error, " > ", y_threshold)
            L_move1(1)
        elif (y_error < -y_threshold):
            print("Right move 1 ", y_error," < ", -y_threshold)
            R_move1(1)

        # 靠近标签
        elif (x_error > x_threshold+0.12):
            print("Moving forward to approach tag", x_error, " > ", x_threshold+0.08)
            go_fast1(1)
        elif (x_error > x_threshold+0.06):
            print("Moving forward to approach tag", x_error, " > ", x_threshold+0.06)
            go_fast(1)
        else:    
            is_turn_done = True 

    return is_turn_done


if __name__ == '__main__':
    rospy.init_node('image_listener')   #ROS节点初始化
    Tag = TagConverter()
    time.sleep(PAUSE_STEP)
    ID = 0      
    step = 1
    level = "start_box"
    time.sleep(5)
    while ChestOrg is None:
        print('wait')
        time.sleep(0.1)

    print('Starting')
    go_fast2(1)

    while not rospy.is_shutdown():
        if ID == 0:  #搬箱子
            if level == "start_box":
                find_box(Chest_img, 'green')
                goto_box()
                time.sleep(PAUSE_LOOP)
            elif level == "end_box":
                if step == 1:
                    box_go(1)
                    BoxR_turn2(1)

                elif step == 2:
                    BoxR_turn2(3)
                    step = 1
                ID += 1
        else:     
            marker = Tag.get_nearest_marker()
            
            if len(marker) == 0:
                print("No tag detected")
                if (ID == 1 and level == "end_box"):
                    print("Turning right")
                    BoxR_turn2(1)
                elif (ID == 1 and level == "start_moving") or ID == 2 or ID == 3 or ID == 4 or (ID == 5 and step == 1):
                    print("Moving backward while holding box")
                    Box_Back(1)
                elif (ID == 5 and step == 2 and level == "start_moving"):
                    print("Turning right")
                    R_turn2(1)
                elif (ID == 6) or (ID == 7) or (ID == 5 and step == 2 and level == "reverse_moving"):
                    print("Moving backward")
                    Back2(1)
                              
            else:
                print(f"Detected AR tag ID: {marker[0]} at position ({marker[1]}, {marker[2]}) with angle {marker[3]}")
                print(marker[0],marker[1],marker[2],marker[3])
                robot_tag_x = marker[1]
                robot_tag_y = marker[2]
                tag_yaw = marker[3] + 90 # artag 正方向与机器人正方向对齐

                if step == 1:
                    if marker[0] == 1:
                        if ID == 1:
                            level = "start_moving"
                            result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.08 , 0.0 , 2)
                            if result == True:
                                print('Tag 1 alignment completed, moving forward to align with tag 2')
                                ID += 1
                                box_go(1)
                                BoxR_move1(1)
                        else:
                            print("Moving forward")
                            box_go(1)

                    elif marker[0] == 2:
                        if ID == 2:
                            result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.12 , 0.0 , 0)
                            if result == True:
                                print('Tag 2 alignment completed, moving right to align with tag 3')
                                ID += 1
                                BoxR_move2(3)
                        else:
                            print("Moving right")
                            BoxR_move2(1)

                    elif marker[0] == 3:
                        if ID == 3:
                            result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.12 , 0.11 , 0)
                            if result == True:
                                print('Tag 3 alignment completed, moving right to align with tag 4')
                                ID += 1
                                BoxR_move2(1)
                                BoxR_move1(1)
                                box_go(2)
                        else:
                            print("Moving right")
                            BoxR_move2(1)
                            

                    elif marker[0] == 4:
                        if ID == 4:
                            result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.12 , -0.1 , 0)
                            if result == True:
                                print('Tag 4 alignment completed, moving left to align with tag 5')
                                ID += 1
                                BoxL_move2(3)
                        else:
                            print("Moving left")
                            BoxL_move2(1)

                    elif marker[0] == 5:
                        if ID == 5:
                            result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.12 , 0.0 , 0)
                            if result == True:
                                print('Tag 5 alignment completed, moving forward to base and putting down sponge block')
                                box_go(3)
                                Box_Down(1)
                                R_turn2(3)
                                step = 2
            
                elif step == 2:
                    #反方向
                    if marker[0] == 5:
                        result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.08 , 0.0 , 180)#5号为反方向
                        level = "reverse_moving"
                        if result == True:
                            print('Tag 5 alignment completed, moving straight to align with tag 6')
                            ID += 1
                            go_fast2(1)
                            go_fast1(2)
                    elif marker[0] == 6:
                        if ID == 6:
                            result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.05 , -0.1 , 0)
                            if result == True:
                                print('Tag 6 alignment completed, moving left to align with tag 7')
                                ID += 1
                                L_move2(4)
                                go_fast2(2)
                                go_fast1(1)
                    
                    elif marker[0] == 7:
                        if ID == 7:
                            result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.06 , 0.10 , 0)
                            if result == True:
                                print('Tag 7 alignment completed, moving right in reverse to align with tag 1')
                                R_move2(5)
                                L_move2(2) #debug
                    elif marker[0] == 1:
                        result = turn_to_tag(robot_tag_x, robot_tag_y, tag_yaw, 0.1 , 0.0 , 180)#1号为反方向
                        if result == True:
                            print('Tag 1 alignment completed, moving forward to grab sponge block')
                            go_fast2(2)
                            ID = 0
                            level = "start_box"
            time.sleep(PAUSE_LOOP)
