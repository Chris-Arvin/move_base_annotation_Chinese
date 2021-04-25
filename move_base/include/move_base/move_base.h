#ifndef NAV_MOVE_BASE_ACTION_H
#define NAV_MOVE_BASE_ACTION_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>  //这个是专门用来处理action的特定的包
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>  //前面那个功能包下的后面那个msg文件
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

//注意，没写using namespace std;

namespace move_base{
    //替换别名，这里是move_base_msgs文件生成的，包含PoseStamped形式的goal和current pose
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
    //3个主要行为，逻辑处理的主要依赖
    enum MoveBaseState{
        PLANNING,
        CONTROLLING,
        CLEARING
    };
    //机器人导航失败时，是哪里出了问题，仅作为显示
    enum RecoveryTrigger{
        PLANNING_R,
        CONTROLLING_R,
        OSCILLATION_R
    };

    class MoveBase{
        public:
            MoveBase(tf2_ros::Buffer& tf);
            virtual ~MoveBase();
            //**最最核心的函数，通过switch来控制程序逻辑
            bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);
            
        private:
            //std_srvs命名空间下的Empty这个struct下的Request
            bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
            //提供给外界，外界可调用本程序的全局规划器
            bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);
            //move_base和全局规划期的过度函数，本质是调用了规划器的makePlan
            bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
            bool loadRecoveryBehaviors(ros::NodeHandle node);
            void loadDefaultRecoveryBehaviors();
            void clearCostmapWindows(double size_x, double size_y);
            void publishZeroVelocity();

            void resetState();
            //这里的输入也可以用geometry_msgs::PoseStamped goal，是一样的，但是浪费内存。
            //boost会为所有的msg自动生成::ConstPtr的新后缀，是boost::shared_ptr形式，共享内存，主要是在delete的时候，
            //这个地址会在所有指向它的引用都消失后，才被delete
            void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
            //*核心函数，规划线程，执行全局路径规划（调用全局路径规划器）
            void planThread();
            //action的回调函数，程序的核心入口，本质是调用了executeCycle
            void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal); //这里如果用MoveBaseGoal::ConstPtr可以吗
            bool isQuaternionValid(const geometry_msgs::Quaternion& q);
            bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);
            double distance(const geometry_msgs::PoseStamped& pl, const geometry_msgs::PoseStamped& p2);
            geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);  //这里为啥没用::ConstPtr
            void wakePlanner(const ros::TimerEvent& event);

            tf2_ros::Buffer& tf_;   //这个tf_主要是用于保存各种state_transformation
            MoveBaseActionServer* as_;  //这个全程是action server，本质上是一个action的容器
            boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;  //局部规划器
            costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;   //全局地图和局部地图
            
            boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
            std::string robot_base_frame_,global_frame_;
            std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>> recovery_behaviors_;
            std::vector<std::string> recovery_behavior_names_;
            
            //执行recovery时，执行哪种类型的recovery，是根据recovery_trigger生成的。但在这个程序中没有被区分，都是从第0个执行到最后一个，把所有的recovery行为都执行掉。。
            unsigned int recovery_index_;
            geometry_msgs::PoseStamped global_pose_;
            //全局规划器的频率，仅在planthread中使用；局部控制器的频率，仅在executeCb中使用，进而控制调用executeCycle的频率，其实control planner和recovery的频率是相同的，path planner的频率由它自己向下控制（注意path planning的频率应该是低于controller planning的）
            double planner_frequency_, controller_frequency_;
            double inscribed_radius_, circumscribed_radius_;
            //全局规划允许的，当规划失败时，再继续尝试规划的容忍时间；局部规划
            double planner_patience_, controller_patience_;
            int32_t max_planning_retries_;  //最大允许尝试进行规划的次数
            uint32_t planning_retries_; //尝试进行规划的次数的计数器
            double conservative_reset_dist_, clearing_radius_;

            //当前目标，仅作为显示； 发送给下位机的速度； MoveBaseAction形式的goal； 是否在进行recovery
            ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
            //和action_goal_pub_一起使用，进行消息格式的转化（用户输入的goal的标准化处理）
            ros::Subscriber goal_sub_;
            ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
            //最初是否关闭两个costmap的更新（即使关闭，在得到goal时，在executeCb中，也会打开更新的）;是否允许机器人原地旋转来进行recovery；是否允许进行recovery
            bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
            bool make_plan_clear_costmap_;      //在进行路径规划时，是否先在全局和局部地图中清空机器人周围的障碍物（设置为free）；
            bool make_plan_add_unreachable_goal_;       //在选定的goal是不可达时，允许将goal少量偏移，再进行规划，允许将该结束作为路径规划的结果进行输出
            double oscillation_timeout_, oscillation_distance_; //振动行为的忍受时间，振动行为的忍受距离。如果在忍受时间内，机器人移动的距离小于忍受距离，则需要执行recovery了
            //程序目前的状态，分为PLANNING, CONTROLLING, CLEARING三种
            //在规划线程中，如果全局路径规划成功则进行CONTROLLING，如果失败次数大于额定次数，则进入CLEARING
            //在主线程中，CONTROLLING会根据失败的尝试时间，将其更改为PLANNING或CLEARING；CLEARING每recovery一次，就将其改为PLANNING
            MoveBaseState state_;
            //这个表征的是recovery是由什么触发的，可能是由于PLANNING,CONTROLLING,CLEARING这三个中的一个触发的
            //这只是一个标志，用于提示用户是哪里出问题了，真正执行的recovery行为都是一样的，按顺序从头到位执行完recovery。
            RecoveryTrigger recovery_trigger_;
            //上一次有效的、可执行的全局规划路径；上一次有效的局部控制；上一次振动结束/冲着的时间
            ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
            geometry_msgs::PoseStamped oscillation_pose_;   //振动标志位，每当当前位置距离上一次振动标志位的距离大于额定距离，则更新其为当前距离
            //这里没看太懂。。是在pluginlib这个namespace下的ClassLoader这个类，它继承自nav_core::xxx？
            pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
            pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
            pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;
            //这三个都是用来保存规划出的全局路径的。
            //规划器首先得到planner_plan_，然后在规划线程中把他传递给latest_plan_（swap，保存），
            //主线程中，latest_plan_传递给controller_plan_（swap，保存），
            //控制器使用control_plan_产生速度
            std::vector<geometry_msgs::PoseStamped>* planner_plan_;
            std::vector<geometry_msgs::PoseStamped>* latest_plan_;
            std::vector<geometry_msgs::PoseStamped>* controller_plan_;

            //和规划线程相关的内容
            //*主线程对第二线程的控制，决定是否进行路径规划
            bool runPlanner_;
            boost::recursive_mutex planner_mutex_;  //规划的线程锁
            boost::condition_variable_any planner_cond_;    //规划的wrapper，用作wait
            //*规划器进行规划时参考的全局goal，是lock的原因所在
            geometry_msgs::PoseStamped planner_goal_;    
            boost::thread* planner_thread_;     //进行规划的线程

            boost::recursive_mutex configuration_mutex_;    //重配置的线程锁。在executeCycle的时候，被拿走
            dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;

            void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

            move_base::MoveBaseConfig last_config_;
            move_base::MoveBaseConfig default_config_;
            //是否初始化了configure;全局规划的频率是否发生了变化；全局规划的频率是否发生了变化。和reconfigureCB有关
            bool setup_, p_freq_change_, c_freq_change_;
            bool new_global_plan_;  //获得新的全局规划，在planThread中被改为true
    };
};
#endif