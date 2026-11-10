// 说明：舵机方向根据实际调整，这里用的占空比0%表示闭合，100%表示全部打开
#include <lib_library.h>
#include "complete_mission.h"
#include <std_msgs/Int32.h>
#include <string>
#include <iostream>
#include <algorithm>
#include "ctrl_trans_msgs/ctrl_mode.h"
// 声明来自complete_mission.h的外部变量，用于重置窗口识别标志位
extern float object_recognize_track_last_time_position_x;
extern float object_recognize_track_last_time_position_y;
extern bool  object_recognize_track_flag;
/*--------------------------------------------------
   全局变量
--------------------------------------------------*/
int  mission_num   = 0;
int  tmp_marker_id = 1;
int  target_qr_id  = 1;  // 当前任务的目标二维码ID

/* Python 节点 qr_land_flag.py 发布的对准完成标志 */
bool qr_land_done_flag = false;
int current_qr_id = 0;

/* 防止重复发布目标ID */
int last_published_target_qr_id = -1;

/* 地面任务控制模式 */
ctrl_trans_msgs::ctrl_mode ctrl_mode_msg;

/*--------------------------------------------------
   任务点坐标表（下标 0/1/2 对应任务点 1/2/3）
--------------------------------------------------*/
const double mission_points[3][2] = {
    { 3.18, -0.09 },   // 任务点 1
    { 3.08, -1.45 },   // 任务点 2
    { 3.06, -2.86 }    // 任务点 3
};

/* 启动时选择的两个开启激光笔的任务点（下标 0/1/2），已按飞行顺序排列 */
int selected_points[2] = { 0, 1 };

/* 判断任务点 idx（0/1/2）是否被选中开启激光笔 */
bool is_laser_point(int idx)
{
    return idx == selected_points[0] || idx == selected_points[1];
}

/*--------------------------------------------------
   启动时选择开启激光笔的两个任务点（三个点都会飞）
   输入 12 / 13 / 23（支持 21、1,3、1 3 等写法，自动按从近到远排序）
--------------------------------------------------*/
void select_mission_points(int argc, char **argv)
{
    std::string input = (argc > 1) ? argv[1] : "";

    while (ros::ok())
    {
        if (input.empty())
        {
            printf("\n三个任务点都会飞，请选择开启激光笔的两个（输入 12 / 13 / 23）：\n");
            for (int i = 0; i < 3; ++i)
                printf("  %d -> (%.2f, %.2f)\n", i + 1,
                       mission_points[i][0], mission_points[i][1]);
            printf("请输入: ");
            fflush(stdout);

            if (!std::getline(std::cin, input))
            {
                /* 无终端输入（例如 roslaunch 启动），默认选中 1、2 */
                printf("\n未检测到键盘输入，默认激光笔开在 1、2 点\n");
                input = "12";
            }
        }

        /* 去掉空白、逗号和全角标点，只保留数字 */
        std::string digits;
        for (char c : input)
        {
            if ((unsigned char)c >= 0x80) continue;   // 全角标点等
            if (c == ' ' || c == '\t' || c == ',') continue;
            digits += c;
        }

        if (digits.size() == 2 &&
            digits[0] >= '1' && digits[0] <= '3' &&
            digits[1] >= '1' && digits[1] <= '3' &&
            digits[0] != digits[1])
        {
            if (digits[0] > digits[1])
                std::swap(digits[0], digits[1]);

            selected_points[0] = digits[0] - '1';
            selected_points[1] = digits[1] - '1';
            ROS_INFO("激光笔将在任务点 %d 和 %d 开启，三个任务点均会飞过",
                     selected_points[0] + 1, selected_points[1] + 1);
            return;
        }

        printf("输入无效：\"%s\"，请输入两个不同的任务点编号（1/2/3），例如 12、13、23\n",
               input.c_str());
        input.clear();
    }
}

/* 时间记录函数实现 */
bool lib_time_record_func(float time_duration, ros::Time time_now)
{
    if (!lib_time_record_start_flag)
    {
        lib_mission_success_time_record = time_now;
        lib_time_record_start_flag = true;
        return false;
    }
    
    if (time_now - lib_mission_success_time_record > ros::Duration(time_duration))
    {
        lib_time_record_start_flag = false;
        return true;
    }
    
    return false;
}

/*--------------------------------------------------
   回调：Python 节点二维码就绪标志
--------------------------------------------------*/
void qr_land_cb(const std_msgs::Bool::ConstPtr &msg)
{
    qr_land_done_flag = msg->data;
}

/*--------------------------------------------------
   回调：Python 节点二维码ID
--------------------------------------------------*/
void qr_id_cb(const std_msgs::Int32::ConstPtr &msg)
{
    current_qr_id = msg->data;
}

/*--------------------------------------------------
   回调：地面任务控制模式
--------------------------------------------------*/
void ctrl_mode_cb(const ctrl_trans_msgs::ctrl_mode::ConstPtr& msg)
{
    ctrl_mode_msg = *msg;
}

int main(int argc, char **argv)
{
    /* 防止中文输出乱码 */
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "complete_mission");

    /* 启动时选择要飞行的两个任务点 */
    select_mission_points(argc, argv);

    ros::NodeHandle nh;

    /*--------------- 订阅 ---------------*/
    ros::Subscriber state_sub      = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub  = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(
        "object_position", 100, object_pos_cb);
    ros::Subscriber cmd_vel_sub    = nh.subscribe(
        "/cmd_vel", 10, cmd_to_px4);

    /* 订阅 Python 节点输出 */
    ros::Subscriber qr_done_sub = nh.subscribe<std_msgs::Bool>(
        "qr_land_done", 10, qr_land_cb);
    ros::Subscriber qr_id_sub = nh.subscribe<std_msgs::Int32>(
        "qr_code_id", 10, qr_id_cb);
    
    /* 订阅地面任务控制模式 */
    ros::Subscriber ctrl_mode_sub = nh.subscribe<ctrl_trans_msgs::ctrl_mode>(
        "/ctrl_mode", 10, ctrl_mode_cb);

    /*--------------- 发布 ---------------*/
    ros::Publisher mavros_setpoint_pos_pub =
        nh.advertise<mavros_msgs::PositionTarget>(
            "/mavros/setpoint_raw/local", 100);
    
    /* 发布目标二维码ID给二维码检测节点 */
    ros::Publisher target_qr_id_pub = 
        nh.advertise<std_msgs::Int32>("/target_qr_id", 10);

    /*--------------- 服务 ---------------*/
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>(
            "mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode");
    ros::ServiceClient ctrl_pwm_client =
        nh.serviceClient<mavros_msgs::CommandLong>(
            "mavros/cmd/command");

    ros::Rate rate(20);

    /* 等待与飞控连接 */
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    /* 起飞前固定 setpoint */
    setpoint_raw.type_mask        = 64 + 128 + 256 + 512;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x       = 0;
    setpoint_raw.position.y       = 0;
    setpoint_raw.position.z       = ALTITUDE;
    setpoint_raw.yaw              = 0;

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    /* 设置模式 / 解锁 */
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    /* 舵机复位 */
    lib_pwm_control(0, 0);
    ctrl_pwm_client.call(lib_ctrl_pwm);

    /******************** 等待地面任务完成 ********************/
    ctrl_mode_msg.ctrl_mode = 0;
    while(ros::ok())
    {
        if(ctrl_mode_msg.ctrl_mode == 1) 
        {
            break;
        }
        printf("ctrl_mode = %d\n", ctrl_mode_msg.ctrl_mode);
        ROS_INFO("等待地面任务完成中...");
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("地面任务完成，开始无人机任务");

    /******************** 起飞阶段 ********************/
    while (ros::ok())
    {
        /* 切换 OFFBOARD */
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        /* 解锁 */
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        /* 到达高度 ALTITUDE 后停 5 秒稳定悬停再进入任务 */
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.1)
        {
            if (ros::Time::now() - last_request > ros::Duration(1.0))
            {
                ROS_INFO("到达目标高度，开始稳定悬停");
                mission_num = 1;
                break;
            }
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    /******************** 主任务状态机 ********************/
    bool       tmp_time_record_start_flag       = false;
    ros::Time  tmp_mission_success_time_record;
    float      tmp_x = 0, tmp_y = 0;
    int        tmp_mission = 1;
    bool       object_data_cleared = false;  // 添加标志位，确保清空旧的识别数据
    
    // 窗口搜索超时保护变量
    ros::Time  window_search_start_time;      // 窗口搜索开始时间

    while (ros::ok())
    {
        if (tmp_mission != mission_num)
        {
            tmp_mission = mission_num;
            printf("mission_num = %d\r\n", mission_num);
        }

        switch (mission_num)
        {
        /*---------------- 任务 1：稳定悬停阶段 ----------------*/
        case 1:
            if (mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2))
            {
                ROS_INFO("稳定悬停完成，准备开始窗口识别");
                object_data_cleared = false;  // 重置清空标志
                window_search_start_time = ros::Time::now();  // 记录搜索开始时间
                mission_num = 2;
            }
            break;

        case 2: /* 窗口识别 */
            if (object_recognize_track("wujiaqi", 0, 1, ALTITUDE, 20, 0.1))
                mission_num = 3;
            break;

        case 3: /* 穿过窗口 */
            if (target_through(2.7, 0.0, ALTITUDE, 0))
                mission_num = 4;
            break;

        /*---------------- 第 1 个任务点（激光笔按启动选择）----------------*/
        case 4:
        {
            const double *p = mission_points[0];
            if (mission_pos_cruise(p[0], p[1], ALTITUDE, 0, 0.2))
            {
                if (!is_laser_point(0))
                {
                    ROS_INFO("第1个任务点未选中，飞过但不开启激光笔");
                    mission_num = 41;
                }
                else
                {
                    lib_pwm_control(100, 100);
                    ctrl_pwm_client.call(lib_ctrl_pwm);
                    // 使用 lib 延时函数，等待 3 秒
                    if (lib_time_record_func(3.0, ros::Time::now()))
                    {
                        // 3秒后关闭激光笔
                        lib_pwm_control(0, 0);
                        ctrl_pwm_client.call(lib_ctrl_pwm);
                        ROS_INFO("第1个任务点完成，关闭激光笔");
                        mission_num = 41;
                    }
                }
            }
            break;
        }
        case 41:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 0, 0.2))
                mission_num = 5;
            break;
        /*---------------- 第 2 个任务点（激光笔按启动选择）----------------*/
        case 5:
        {
            const double *p = mission_points[1];
            if (mission_pos_cruise(p[0], p[1], ALTITUDE, 0, 0.2))
            {
                if (!is_laser_point(1))
                {
                    ROS_INFO("第2个任务点未选中，飞过但不开启激光笔");
                    mission_num = 51;
                }
                else
                {
                    lib_pwm_control(100, 100);
                    ctrl_pwm_client.call(lib_ctrl_pwm);
                    // 使用 lib 延时函数，等待 3 秒
                    if (lib_time_record_func(3.0, ros::Time::now()))
                    {
                        // 3秒后关闭激光笔
                        lib_pwm_control(0, 0);
                        ctrl_pwm_client.call(lib_ctrl_pwm);
                        ROS_INFO("第2个任务点完成，关闭激光笔");
                        mission_num = 51;
                    }
                }
            }
            break;
        }
        case 51:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 0, 0.2))//误差似乎可以适当调小
                mission_num = 6;
            break;
        /*---------------- 第 3 个任务点（激光笔按启动选择）----------------*/
        case 6:
        {
            const double *p = mission_points[2];
            if (mission_pos_cruise(p[0], p[1], ALTITUDE, 0, 0.2))
            {
                if (!is_laser_point(2))
                {
                    ROS_INFO("第3个任务点未选中，飞过但不开启激光笔");
                    mission_num = 61;
                }
                else
                {
                    lib_pwm_control(100, 100);
                    ctrl_pwm_client.call(lib_ctrl_pwm);
                    // 使用 lib 延时函数，等待 3 秒
                    if (lib_time_record_func(3.0, ros::Time::now()))
                    {
                        // 3秒后关闭激光笔
                        lib_pwm_control(0, 0);
                        ctrl_pwm_client.call(lib_ctrl_pwm);
                        ROS_INFO("第3个任务点完成，关闭激光笔");
                        mission_num = 61;
                    }
                }
            }
            break;
        }
        case 61:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 0, 0.2))
                mission_num = 71;
            break;
        /*---------------- 出窗 ----------------*/
        case 71:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 3.14, 0.2))//转向，同时误差回调防止原地卡自己悬停
                mission_num = 711;
            break;
        
        case 711: /* 第二次穿窗前稳定悬停 */
            if (lib_time_record_func(2.0, ros::Time::now()))  // 稳定悬停1秒，悬停时间太长容易漂
            {
                ROS_INFO("第二次穿窗前稳定悬停完成，开始窗口识别");
                // 重置窗口识别标志位，确保第二次识别能重新初始化
                object_recognize_track_flag = false;
                ROS_INFO("重置窗口识别标志位，准备第二次窗口识别");
                mission_num = 72;
            }
            break;
        case 72:
            if (object_recognize_track("wujiaqi", 3.14, -1, ALTITUDE, 20, 0.1))
            {
                // 重置穿窗标志位，确保第二次穿窗能重新初始化位置
                extern bool target_through_init_position_flag;
                target_through_init_position_flag = false;
                ROS_INFO("重置穿窗标志位，准备第二次穿窗");
                mission_num = 73;//在第二个窗口识别到就直接降落
            }
            break;
        case 73: /* 第二次穿窗 */
            {
                static bool timer_started = false;
                static ros::Time start_time;
            
                if (!timer_started) {
                    start_time = ros::Time::now();
                    timer_started = true;
                    ROS_INFO("进入case 73，3秒后将强制进入mission_num=8");
                }
            
                // 执行穿窗动作
                bool through_done = target_through(-2.7, 0, ALTITUDE, 3.14);
            
                // 满足任意条件即进入下一阶段
                if (through_done || ros::Time::now() - start_time > ros::Duration(3.0)) {
                    mission_num = 8;     // 进入下一阶段
                    timer_started = false;
                    target_through_init_position_flag = false;  // 重置穿窗标志，防止下一次出错
                    ROS_INFO("穿窗成功或超时3秒，进入mission_num=8");
                }
            }
            break;
        /*---------------- 终点 1 ----------------*/
        case 8:
            if (mission_pos_cruise(0, 0, ALTITUDE, 3.14,0.2))//需要在终点降落
            {
                mission_pos_cruise_flag = false;
                mission_num            = 81;
            }
            break;
        case 81:
            if (current_position_cruise(0, 0, ALTITUDE, 3.14, 0.2))
                mission_num = 9;//到达终点直接降落
            break;
        case 9: /* 再下降 0.2 m */
            if (mission_pos_cruise(0, 0, -0.2, 3.14, 0.2) &&
                lib_time_record_func(1.0, ros::Time::now()))//tmp_x,tmp_y继承自识别到二维码的位置
                mission_num = 10;
            break;
        case 10: /* AUTO.LAND */
            ROS_INFO("AUTO.LAND");
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
            mission_num = 10;
            break;
        }

        /* 发布 setpoint */
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
