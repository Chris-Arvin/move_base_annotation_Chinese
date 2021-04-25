/**
* maintainer: Arvin
* Email:1711506@mail.nankai.edu.cn
**/

#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base{
    /**
    *   MoveBase的构造函数，除了定义了一堆东西外，有如下核心内容：
    *   开启的server(action, service)【工程的入口，本质上通过service的callback进行，工程的入口是1】
    *   1. * 开启了/move_base这个actionserver，回调函数是executeCb()，进一步调用executeCb()，核心内容，这是程序的主要入口。它是被MoveBaseAction形式的名为"goal"的topic唤醒的。
    *   2. * 开启了/move_base/make_plan这个server，回调函数是planService()，用于接收外界提供的goal，并进行全局路径规划。注意，这个函数只是给外界提供了调用全局路径规划的接口，并不参与此工程，可删。
    *   3. 开启了/move_base/clear_costmaps这个server，回调函数是clearCostmapsService()，用于接收是否清空地图
    *   
    *   定义的topic
    *   1. /move_base/current_goal 输出当前goal，但我不知道为啥它的队列长度是0，应该是不需要被读取，只用作显示？一去不回~
    *   2. * /move_base/goal，注意这里有两个同名的topic，一个是PoseStamped格式的subscriber，用于接收用户给的goal，另一个用于发布MoveBaseActionGoal形式的goal。这个转换可以唤醒executeCb()，即工程的主入口
    *   3. /move_base/recovery_status 输出表征是否进行了recovery
    * 
    *   其他处理
    *   1. 定义了planner_thread_，开启了规划线程，该线程对应的函数是planThread()
    *   2. 开启了全局地图和局部地图的更新，【这里也开启了两个新的线程】
    *   3. 开启了dynamic reconfigure
    *   4. state_=PLANNING，初始化state_为正在进行路径规划
    *   5. recovery_index_开始计数，到达一定次数后，进行recovery_behavior
    * 
    *   全局规划器(planner_)使用到的函数：
    *       initialize()
    *       makeplan()
    *   局部规划器使用到的函数：
    *       initialize()
    *       setPlan()
    *       isGoalReached()
    *       computeVelocityCommands()
    * 
    *   接口：
    *   1. 与地图的接口是通过costmap_ros实现的，在程序内部通过建立两个类并开始对应的新线程来实现
    *   2. 与下位机的电机接口是通过/cmd_vel这个topic来实现的(executeCycle函数)
    *   3. 机器人的位置是通过读取tf2实现的（getRobotPose函数）
    * 
    *   note
    *   1. boost::bind是函数绑定期，核心优势在于占位符的使用。在绑定成员函数时，注意&和指针(一般为this)的使用
    *   2. boost::recursive_mutex aa是创建线程锁，用boost::unique_lock<boost::recursive_mutex> bb(aa)来先拿一下锁，锁住，当生命周期结束后，自动销毁！
    *       用boost::condition_variable_any cc, cc.wait(bb)来等待拿锁，第二个参数默认为false，进行等待，知道有人调用cc.notify_one()时才变成true，继续往下执行。
    *   3. ros::NodeHandle sth是句柄，可以理解为创建门把手，这样不管门是怎样的都比较好开。当直接... nh时，nh下创建的话题名称都是/topic_name。如果是... nh("~")，则nh下创建的是/node_name/topic_name （这里的nh一般用private_nh），
    *       其他的例如... nh("aa")，则对应的是/aa/topic_name。
    *   4. 用bool做函数类型，用引用来赋值，这工程写的真漂亮。Eitan nb！！
    * 
    **/
    MoveBase::MoveBase(tf2_ros::Buffer& tf):
        tf_(tf),
        as_(NULL),
        planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
        bgp_loader_("nav_core","nav_core::BaseGlobalPlanner"),
        blp_loader_("nav_core","nav_core::BaseLocalPlanner"),
        recovery_loader_("nav_core","nav_core::RecoveryBehavior"),
        planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
        runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false)
        {
            //这是一个加强版的server，周期性地给feedback，名字为"move_base"，可以由我们自己控制开启和关闭
            //注意，它这个action，只有feedback，没有result
            as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

            ros::NodeHandle private_nh("~");    //这个node接收的是自己的局部变量，它接收的参数都是node_name/param_name
            ros::NodeHandle nh;     //这个node接收的是全局变量，它接收的参数是param_name

            recovery_trigger_ = PLANNING_R;
            
            //用private_nh接收一些局部变量（他们在MoveBase.cfg中被赋值）
            std::string global_planner, local_planner;
            //接收的param name（本质上是node_name/param_name），程序中将会被赋值的参数，如果param池中没有则会被使用的默认值
            //详细解释可参照move_base/cfg/MoveBase.cfg
            private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));//第一个实际上接收的是move_base_node/base_global_planner
            private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
            private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
            private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
            private_nh.param("planner_frequency", planner_frequency_, 0.0);
            private_nh.param("controller_frequency", controller_frequency_, 20.0);
            private_nh.param("planner_patience", planner_patience_, 5.0);
            private_nh.param("controller_patience", controller_patience_, 15.0);
            private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

            private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
            private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

            // parameters of make_plan service
            private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
            private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);
            
            planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();   //为什么control的也是poseStamped

            //设置并直接开始规划线程
            //boost::bind是函数绑定器，核心在于它使用占位符很厉害，而在bind+成员函数时，必须用&+this（某个类的指针）的形式
            //参照 https://www.cnblogs.com/blueoverflow/p/4740093.html
            planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

            vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
            current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal",0); //这儿为啥是0啊

            ros::NodeHandle action_nh("move_base");
            action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal",1);
            recovery_status_pub_ = action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status",1);

            //这里NodeHandle和对应的goalCB是做格式转换的，把PoseStamped格式的信息转化为MoveBaseActionGoal，发布到同名但不同类型的另一个"goal"，nb！
            ros::NodeHandle simple_nh("move_base_simple");
            goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal",1,boost::bind(&MoveBase::goalCB, this, _1));
            
            //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
            private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
            private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
            private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
            private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

            private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
            private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
            private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);            

            //和全局地图相关的wrapper，这个类里面，其实也创建了新的线程。
            planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);     //global_costmap是这个map的名字。这个tf_的具体内容是啥啊，好像是图间的转换。
            planner_costmap_ros_->pause();       
            try{
                planner_ = bgp_loader_.createInstance(global_planner);
                ROS_INFO("global planner created");
                planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);    //这个getNmae做了split，取/后面的内容，此文件默认的global_planner=navfn/NavfnROS，这里取的就是NavfnROS
            } catch(const pluginlib::PluginlibException& ex){
                ROS_FATAL("g lobal_planner failed");
                exit(1);
            }

            //和局部地图相关的wrapper，这个类，依然创建了自己的线程。
            controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap",tf_);
            controller_costmap_ros_->pause();
            try{
                tc_ = blp_loader_.createInstance(local_planner);
                ROS_INFO("local planner created");
                tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
            } catch(const pluginlib::PluginlibException& ex){
                ROS_FATAL("local_planner failed");
                exit(1);
            }

            //开启地图的更新
            planner_costmap_ros_->start();
            controller_costmap_ros_->start();

            make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);
            clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

            //是否有停止地图更新的指令
            if(shutdown_costmaps_)
            {
                ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
                planner_costmap_ros_->stop();
                controller_costmap_ros_->stop();
            }

            //先尝试加载我们提供的recovery方式，如果失败，则加载默认的
            if(!loadRecoveryBehaviors(private_nh)){
                loadDefaultRecoveryBehaviors();
            }

            //当前状态是PLANNING
            state_ = PLANNING;

            recovery_index_ = 0;

            //开始做事啦
            as_->start();

            //和动态参数配置相关的server
            dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
            dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);     
    }


    /** 
     * 消息格式转换，接收用户的posestamped形式的goal，发送MoveBaseActionGoal形式的goal，唤醒executeCb()
    **/
    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
        ROS_DEBUG_NAMED("move_base","transfer user's goal from 'PoseStamped' to 'MoveBaseAction'.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;
        action_goal_pub_.publish(action_goal);
    }

    /**
     * 只关注path planning部分，给定起点和终点，返回一条【路径】.如果给定的goal不可达，则在周围寻找一个subgoal
     * 当我们自己写了一个path planning的算法且作为插件加载进来时，可以用这个service来做测试
     * */
    bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
    {
        //as_正在进行实施动态规划，不能在此时被调用。
        if(as_->isActive())
        {
            ROS_ERROR("move_base is occupied.");
            return false;
        }
        //找不到全局地图
        if(planner_costmap_ros_==NULL)
        {
            ROS_ERROR("global costmap is not defined");
            return false;
        }

        //如果在call中定义的start是空的，那么默认赋值为机器人的当前位置
        geometry_msgs::PoseStamped start;
        if(req.start.header.frame_id.empty())
        {
            geometry_msgs::PoseStamped global_pose;
            if(!getRobotPose(global_pose, planner_costmap_ros_))
            {
                ROS_ERROR("cannot find start pose");
                return false;
            }
            start = global_pose;
        }
        else{
            start = req.start;
        }

        //如果true，先在全局和局部地图中，清空机器人附近的障碍物点（设置为free），默认为true
        if(make_plan_clear_costmap_)
        {
            clearCostmapWindows(2*clearing_radius_, 2*clearing_radius_);
        }

        //如果能根据goal直接找到global_plan，则直接跳到最后进行赋值，否则需要用subgoal去替代goal
        std::vector<geometry_msgs::PoseStamped> global_plan;
        if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
        {
            ROS_DEBUG_NAMED("move_base","Failed to find a global path. Try subgoal.");

            geometry_msgs::PoseStamped p;
            p = req.goal;
            bool found_legal = false;   //终止标志，是否找到合法解，下面的for的终止标准的写法还挺有意思，可借鉴
            float resolution = planner_costmap_ros_->getCostmap()->getResolution(); //地图分辨率
            float search_increment = resolution*3.0; //搜索的步长
            //修改search_increment
            if(req.tolerance>0.0 && req.tolerance<search_increment) search_increment = req.tolerance;
            //更改x_offset/y_offset是偏移的值/距离，x_mult/y_mult是偏移的方向(-1,0,1)
            for(float max_offset = search_increment; max_offset<req.tolerance&&!found_legal; max_offset+=search_increment)
            {
                for(float y_offset=0; y_offset<=max_offset&&!found_legal; y_offset+=search_increment)
                {
                    for(float x_offset=0; x_offset<max_offset&&!found_legal; x_offset+=search_increment)
                    {
                        if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;
                        for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) 
                        {
                            if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;
                            for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) 
                            {
                                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;
                                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                                if(planner_->makePlan(start, p, global_plan))
                                {
                                    if(!global_plan.empty())
                                    {
                                        if (make_plan_add_unreachable_goal_) 
                                        {
                                            global_plan.push_back(req.goal);
                                        }

                                        found_legal = true;
                                        ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                        break;
                                    }
                                }
                                else{
                                    ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                }
                            }
                        }
                    }
                }
            }
        }

        //将global_plan赋值给resp
        resp.plan.poses.resize(global_plan.size());
        for(unsigned int i=0; i<global_plan.size(); ++i)
        {
            resp.plan.poses[i] = global_plan[i];
        }
        return true;
    }

    /**
     * 核心server的callback，进行了goal的初始化，并首次唤醒了规划线程
     * 在while中，按照*control frequency*进行：
     *      一直在检测频率是否产生更新、四元数和合法性以及地图的frame是否发生了变化。一但有问题，则唤醒规划线程，重新进行规划（注意，只有规划线程才能让state_进行controlling状态！）
     *      在while的最后调用executeCycle根据state_进行switch，决定机器人当前应执行的行为
     * 
     * 退出方法：四元数不合法、goal不被as接受、有人kill了线程，则错误退出；
     *         executeCycle返回true（机器人到达目标点/控制器没有被正常初始化，机器人停止），则正确退出。
     * 
     * ** notify更像是让规划线程不阻塞，并不是每notify一次，就得到一个全局路径！
     * */
    void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
    {
        /**
         * 如果目标的四元数不合法，直接退出
         * */
        if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(),"Aborting on goal because it was sent with an invalid quaternion");
        }
        return;

        /**
         * goal的初始化，首次唤醒规划线程
         * */
        //每当获得一个新的目标，先把goal转到全局坐标系下，并让机器人停止移动
        geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
        publishZeroVelocity();
        //唤醒规划线程！
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();
        //发布当前goal
        current_goal_pub_.publish(goal);
        std::vector<geometry_msgs::PoseStamped> global_plan;
        //* 控制器/本函数执行的频率
        ros::Rate r(controller_frequency_);
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
            planner_costmap_ros_->start();
            controller_costmap_ros_->start();
        }
        //冗余更新时间戳，后续还会被再次更新的
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;  //全局规划的失败/重新尝试次数为0
        
        /**
         * 不出意外的话，如果没人kill这个handle，那么会一直执行下面这个while
         * */
        ros::NodeHandle n;
        while(n.ok())
        {
            //如果控制器的频率发生变化(由reconfigure更改)，则更改。
            if(c_freq_change_)
            {
                ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
                r = ros::Rate(controller_frequency_);
                c_freq_change_ = false;
            }

            //检测goal的合法性，并进行唤醒规划线程
            if(as_->isPreemptRequested()){
                if(as_->isNewGoalAvailable()){
                    move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
                    //检测goal的四元数是否合法
                    if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
                    {
                        as_->setAborted(move_base_msgs::MoveBaseResult(),"Aborting on goal because it was sent with an invalid quaternion");
                        return;
                    }
                    //将goal转化到全局坐标系
                    goal = goalToGlobalFrame(new_goal.target_pose);
                    recovery_index_ = 0;    //恢复次数重置
                    state_ = PLANNING;      //初始状态为PLANNING
                    //确保planner被唤醒的（实际上做初始化的时候就已经唤醒过了）
                    lock.lock();
                    planner_goal_ = goal;
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                    lock.unlock();
                    //发布当前的goal
                    ROS_DEBUG_NAMED("move_base","move_base has received a global of x:%.2f, y:%.2f", goal.pose.position.x, goal.pose.position.y);
                    current_goal_pub_.publish(goal);
                    //重置当前时间
                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;  //规划次数重置
                }
                else{
                    //错误退出，没有获得有效的goal
                    resetState();
                    ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
                    as_->setPreempted();
                    return;
                }
            }

            //检测全局地图的frame是否发生了变化，如果发生了变化，那么赶紧改goal所对应的frame，并重置所有
            if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
            {
                goal = goalToGlobalFrame(goal);
                recovery_index_ = 0;
                state_ = PLANNING;

                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();

                ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y:.%.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
                current_goal_pub_.publish(goal);

                last_valid_control_ = ros::Time::now();
                last_valid_plan_ = ros::Time::now();
                last_oscillation_reset_ = ros::Time::now();
                planning_retries_ = 0;
            }

            /**
             * 核心，调用executeCycle确定接下来该干什么啦
             * */
            ros::WallTime start = ros::WallTime::now();
            bool done = executeCycle(goal, global_plan);
            if(done)
                return;

            //输出并警告这一个cycle的时间是否符合对应的频率
            ros::WallDuration t_diff = ros::WallTime::now() - start;
            ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
            r.sleep();
            if(r.cycleTime() > ros::Duration(1/controller_frequency_) && state_ == CONTROLLING)
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());


        }
        //如果有人kill了这个节点，才会到这里，正常情况下是到不了这里的。
        //打开了规划线程，让executeCb这个控制线程干净的退出（exit cleanly）
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(),"Aborting on the goal because the node has been killed");
        return;
    }

    /**
     * 此函数为一个执行周期，内部无循环，仅会被executeCb调用。
     * 这个函数是本工程文件的最核心的部分，在做逻辑处理。
     * 1. 做action的feedback
     * 2. 更新oscillation的记录时间，确定地图是否正常
     * 3. 检测是否具有新的global plan，如果有的话，更新control_plan，保证控制器参考的路径是最新的。
     * 4. switch：
     *    PLANNING: 唤醒规划线程进行规划；
     *    CONTROLLING: 检测是否到达目标点；检测振动；调用控制器规划速度，如果失败，根据尝试次数，更改state_为PLANNING或CLEARING；
     *    CLEARING: 按照recovery_index恢复一次，此时唤醒规划线程，state_=PLANNING,且该index++；如果所有index都执行完了，则错误退出
     * 
     * 跳出规则：
     * 1. false：地图时间不对；控制器初始化存在问题，正常执行完switch；
     * 2. true：到达目标点，recovery全部执行完（依然找不到速度）
     * */
    bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
        //拿reconfigure的锁，此时不允许重规划了
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        //将被引用赋值，机器人输出的速度，会被发到/cmd_vel
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::PoseStamped global_pose;
        getRobotPose(global_pose, planner_costmap_ros_);
        //仔细想想，这种方式其实就是同名替换，这种写法，要学习一下
        const geometry_msgs::PoseStamped& current_position = global_pose;
        //这个publish不完全是topic那种形式，而是对应着返回feedback【自适应topic】
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_->publishFeedback(feedback);

        //这里的核心意思是，如果距离上次设置的振动位置的距离够远了，则会更新振动标记位，将上次的振动时间更新到现在
        //核心是更改上次振动的时间last_oscillation_reset_，这个变量会在case CONTROLLING中被调用
        //简单解释：如果在限定时间内，机器人走过的距离很小，那么它应该是在原地/短距离内振荡，这是不好的，需要执行recovery了
        if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
            //设置上次oscillation的时间，在switch的case CONTROLLING中会被调用
            last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position;
            if(recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        //确定地图地图时间是否正常，如果不正常，则直接跳出本次cycle
        if(!controller_costmap_ros_->isCurrent())
        {
            ROS_WARN("[%s]: Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }

        //每当规划线程得到一个新的plan，就把latest_plan_更新到controller_plan_，进而更新tc_->setPlan
        if(new_global_plan_)
        {
            new_global_plan_ = false;
            ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");
            std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;
            //锁线程，更新control_plan_（swap）
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            controller_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base","pointers swapped!");
            //更新局部规划器的参考路径，如果失败，让机器人停止，cycle结束。
            //注意这里返回的是true，即控制器因没有被初始化成功等问题，无法继续执行任务，会知道整个executeCb的结束
            if(!tc_->setPlan(*controller_plan_))
            {
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                lock.lock();
                runPlanner_ = false;
                lock.unlock();

                as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
                return true;
            }

            if(recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }

        switch(state_){
            /**
             * 如果是PLANNING状态，那么就去规划就好了，没什么好说的
             * */
            case PLANNING:
                //这个中括号，纯粹是为了处理lock的生命周期。如果自己加unlock，可以去掉中括号。
                {
                    boost::recursive_mutex::scoped_lock lock(planner_mutex_);
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                }
                ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
                break;
            
            /**
             * 如果是CONTROLLING状态：
             * 1. 如果已经到达目标，关闭规划线程，返回true，结束控制规划
             * 2. 如果振动时间很长了（限定时间内移动的距离很小），则更改state_为CLEARING
             * 3. 调用控制器：如果成功，则直接发布；如果失败，则根据尝试次数，将state_更改为CLEARING进行recovery，或唤醒规划线程，state_更改为PLANNING
             * */
            case CONTROLLING:
                ROS_DEBUG_NAMED("move_base","In controlling state.");
                //如果已经到达目标点，则停止全局规划线程，返回true，跳出executeCycle，进而跳出executeCb(也就停止了局部规划器)，直接等待下一个goal再来，再重新开启就好啦
                if(tc_->isGoalReached())
                {
                    resetState();
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();
                    as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                    return true;
                }

                //如果机器人在限定时间内移动的距离很短，那么停止，并进行recovery
                //这里要结合本函数最开始的部分一起看
                if(oscillation_timeout_ > 0.0 && last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
                {
                    publishZeroVelocity();
                    state_ = CLEARING;
                    recovery_trigger_ = OSCILLATION_R;
                }

                //如果控制器可以成功生成速度，则直接发布，让下位机执行，并且重置recovery计数器；
                //否则，根据规划器的尝试次数，重新记性控制规划，或唤醒全局规划。
                {
                    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
                    if(tc_->computeVelocityCommands(cmd_vel))
                    {
                        ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
                        last_valid_control_ = ros::Time::now();
                        vel_pub_.publish(cmd_vel);
                        if(recovery_trigger_ == CONTROLLING_R)
                            recovery_index_ = 0;
                    }
                    else{
                        ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                        //规划器的尝试时间限制，如果超出最大时间了，则让机器人停止，进行recovery
                        ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
                        if(ros::Time::now()>attempt_end)
                        {
                            publishZeroVelocity();
                            state_ = CLEARING;
                            recovery_trigger_ = CONTROLLING_R;
                        }
                        //如果没超出，那么意味着在此global plan下，控制器没法找到速度，需要让机器人停止，并唤醒规划线程重新进行规划
                        else
                        {
                            last_valid_plan_ = ros::Time::now();
                            planning_retries_ = 0;
                            state_ = PLANNING;
                            publishZeroVelocity();
                            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                            runPlanner_ = true;
                            planner_cond_.notify_one();
                            lock.unlock();
                        }
                    }
                }
                break;

            /**
             * 按找recovery_index指示的位置，执行一次recovery行为，并讲recovery_index++
             * 如果所有的recovery都被执行过了，那么结束主线程和规划线程，即此goal下，导航失败。
             * 用ROS_ERROR输出是哪里出错了
             * */
            case CLEARING:
                ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
                //在整个recovery的buffer中，根据recovery_index_选择一个 进行恢复行为。
                //每次到了CLEARING，都只执行一个，然后recovery_index_++，直到recovery全部执行完
                if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
                {
                    ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());
                    //发布进行规划的行为给外界看
                    move_base_msgs::RecoveryStatus msg;
                    msg.pose_stamped = current_position;
                    msg.current_recovery_number = recovery_index_;
                    msg.total_number_of_recoveries = recovery_behaviors_.size();
                    msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];
                    recovery_status_pub_.publish(msg);
                    //在程序内部调用插件，进行recovery
                    recovery_behaviors_[recovery_index_]->runBehavior();
                    //更改振动计时器
                    last_oscillation_reset_ = ros::Time::now();
                    ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
                    last_valid_plan_ = ros::Time::now();
                    //state切换回planning
                    planning_retries_ = 0;
                    state_ = PLANNING;
                    //注意这个，它只执行了一个recovery，如果这次不行，会继续往下执行
                    recovery_index_++;
                }
                //如果所有的recovery行为都执行完，那么说明这个goal下，机器人不能导航。
                //此时，锁住规划线程，并用ROS_ERROR报出是哪里的问题，并且返回true，结束此goal下的主线程规划
                else
                {
                    ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    ROS_DEBUG_NAMED("move_base_recovery", "something should abort after this");
                    if(recovery_trigger_ == CONTROLLING_R){
                        ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
                    }
                    else if(recovery_trigger_ == PLANNING_R){
                        ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
                    }
                    else if(recovery_trigger_ == OSCILLATION_R){
                        ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
                    }
                    resetState();
                    return true;
                }
                break;
            //正常情况下，应该到不了这里（不正常情况下，应该也到不了。。。）
            default:
                ROS_ERROR("This case should never be reached, something is wrong, aborting");
                resetState();
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();
                as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. THis is a bug, please report it.");
                return true;
        }

        return false;

    }






    /**
     * 独立于主线程之外的另一个线程，本质上是不断的根据给定的目标点去进行全局规划
     * 通过第二个while来暂停规划，wait_for_wake是满足自身的规划频率要求；runPlanner是影响主线程的规划器，只有主线程允许进行规划，它才规划呢
     * planner_cond_.wait(lock)好像纯碎是为了防止影响主线程做事【这里还需要再看】
     * 
     * 
     * 主线程允许进行规划时，调用makePlan进行规划
     * 1. 如果规划成功，则更新latest_plan_，供控制器使用，并确认是否需要继续进行路径规划（确认主线程要求和预设的自己的规划频率）
     * 2. 如果规划失败，则跳回去再次尝试。当尝试次数或时间超限时，调用recovery
     * */
    void MoveBase::planThread()
    {
        ROS_DEBUG_NAMED("move_base_plan_thread","Strating planner thread...");
        ros::NodeHandle n;
        ros::Timer timer;
        bool wait_for_wake = false; //这个变量仅仅针对planner_frequency_>0的情况，是为了做等待的。
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_); //先锁住
        while(n.ok())
        {
            while(wait_for_wake || !runPlanner_)    //第一个是需要等待频率，第二个是主线程是否允许开启了规划线程
            {
                ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
                planner_cond_.wait(lock);   //注意，这个wait在执行时，会先unlock，并且它会导致这个线程被压入等待栈，notify_one()是唤醒栈中的一个，notify_all()是唤醒所有的
                wait_for_wake = false;  
            }

            ros::Time start_time = ros::Time::now();
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");
            //正常尝试去获得全局规划的路径
            planner_plan_->clear();
            bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
            //如果能正常规划出路径，则更新latest_plan_，再确认是否进行进行规划
            if(gotPlan){
                ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!",planner_plan_->size());
                std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

                lock.lock();
                //这里做了一个swap，latest_plan_保存的永远是最新规划出来的路径，是会被controller处理的；planner_plan_保存的应该是上一次规划出的（稳定的）路径
                planner_plan_ = latest_plan_;
                latest_plan_ = temp_plan;
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;      //尝试进行规划的次数，如果成功了，肯定就置0了嘛
                new_global_plan_ = true;    
                ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

                if(runPlanner_) //只要有了全局路径，就把state切换到CONTROLLING状态，要进行控制规划啦。注意这里即使变了state_,只要主线程不该runPlanner_，它就一直跑，一直尝试规划
                    state_ = CONTROLLING;
                if(planner_frequency_<=0)   //这里，如果规划频率<=0，则planThread会被卡在第二个while，直到被主线程运行进行规划
                    runPlanner_ = false;
                lock.unlock();
            }
            //如果规划失败，则重新进行尝试，当尝试时间或次数大于限定时，跳出，机器人停止并进行recovery
            else if(state_ == PLANNING){
                ROS_DEBUG_NAMED("move_base_plan_thread","No Plan");
                ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);
                lock.lock();
                planning_retries_ ++;
                if(runPlanner_ && (ros::Time::now()>attempt_end || planning_retries_>uint32_t(max_planning_retries_))){
                    state_ = CLEARING;
                    runPlanner_ = false;
                    publishZeroVelocity();
                    recovery_trigger_ = PLANNING_R; //recovery是因为全局规划失败而出发的
                }

                lock.unlock();
            }

            //为了下一个iteration做准备，因为每个iteration进入的时候都是先unlock的
            lock.lock();

            //这里纯粹是为了满足规划频率的要求
            //如果planner_frequency_<=0，那么这个线程会被直接开在第二个while的；如果>0，则需要用如下的方式来让这个线程卡一会
            //注意createTimer和ros::rate不太一样，或者是卡主整个线程，前者是让线程继续走，但是在sleep_time后调用callback，这个逻辑还挺有意思的。
            if(planner_frequency_>0)
            {
                ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
                if(sleep_time > ros::Duration(0.0))
                {
                    wait_for_wake = true;
                    timer = n.createTimer(sleep_time, & MoveBase::wakePlanner, this);
                }
            }
        }

    }
    
    /**
     * 给定goal，读取机器人当前pose，引用赋值plan
     * 这里先拿了全局地图的锁，停止了它的更新，结束后放开了。
     * 核心是调用了全局规划器的makePlan函数。
     * */
    bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        //停止全局地图的更新
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));
        plan.clear();
        //确认全局地图存在
        if(planner_costmap_ros_ == NULL){
            ROS_ERROR("Global costmap is NULL");
            return false;
        }
        //确认可获得机器人当前pose
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose,planner_costmap_ros_)){
            ROS_WARN("Unable to get the starting pose of robot");
            return false;
        }
        //调用planner的makePlan进行规划
        const geometry_msgs::PoseStamped& start = global_pose;
        if(!planner_->makePlan(start, goal, plan) || plan.empty()){
            ROS_DEBUG_NAMED("move_base","Failed to make plan");
            return false;
        }

        return true;

    }


    void MoveBase::clearCostmapWindows(double size_x, double size_y){
        geometry_msgs::PoseStamped global_pose;

        //clear the planner's costmap
        getRobotPose(global_pose, planner_costmap_ros_);

        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

        //clear the controller's costmap
        getRobotPose(global_pose, controller_costmap_ros_);

        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
    }

    bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
        //clear the costmaps
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }

    MoveBase::~MoveBase(){
        recovery_behaviors_.clear();

        delete dsrv_;

        if(as_ != NULL)
        delete as_;

        if(planner_costmap_ros_ != NULL)
        delete planner_costmap_ros_;

        if(controller_costmap_ros_ != NULL)
        delete controller_costmap_ros_;

        planner_thread_->interrupt();
        planner_thread_->join();

        delete planner_thread_;

        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;

        planner_.reset();
        tc_.reset();
    }

    void MoveBase::publishZeroVelocity(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }

    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
        //first we need to check if the quaternion has nan's or infs
        if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
        ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
        return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        //next, we need to check if the length of the quaternion is close to zero
        if(tf_q.length2() < 1e-6){
        ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
        return false;
        }

        //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if(fabs(dot - 1) > 1e-3){
        ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
        return false;
        }

        return true;
    }

    geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        geometry_msgs::PoseStamped goal_pose, global_pose;
        goal_pose = goal_pose_msg;

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.header.stamp = ros::Time();

        try{
        tf_.transform(goal_pose_msg, global_pose, global_frame);
        }
        catch(tf2::TransformException& ex){
        ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
            goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
        return goal_pose_msg;
        }

        return global_pose;
    }

    void MoveBase::wakePlanner(const ros::TimerEvent& event)
    {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }

    double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
    {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time(); // latest available
        ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

        // get robot pose on the given costmap frame
        try
        {
        tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException& ex)
        {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
        }
        catch (tf2::ConnectivityException& ex)
        {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
        }
        catch (tf2::ExtrapolationException& ex)
        {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
        {
        ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                            "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                            current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
        return false;
        }

        return true;
    }

    bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
        XmlRpc::XmlRpcValue behavior_list;
        if(node.getParam("recovery_behaviors", behavior_list)){
        if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
            for(int i = 0; i < behavior_list.size(); ++i){
            if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
                //check for recovery behaviors with the same name
                for(int j = i + 1; j < behavior_list.size(); j++){
                    if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                    if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                        std::string name_i = behavior_list[i]["name"];
                        std::string name_j = behavior_list[j]["name"];
                        if(name_i == name_j){
                        ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                            name_i.c_str());
                        return false;
                        }
                    }
                    }
                }
                }
                else{
                ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                return false;
                }
            }
            else{
                ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                    behavior_list[i].getType());
                return false;
            }
            }

            //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
            for(int i = 0; i < behavior_list.size(); ++i){
            try{
                //check if a non fully qualified name has potentially been passed in
                if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
                std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                for(unsigned int i = 0; i < classes.size(); ++i){
                    if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                    ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                        std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                    behavior_list[i]["type"] = classes[i];
                    break;
                    }
                }
                }

                boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                //shouldn't be possible, but it won't hurt to check
                if(behavior.get() == NULL){
                ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                return false;
                }

                //initialize the recovery behavior with its name
                behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
                recovery_behavior_names_.push_back(behavior_list[i]["name"]);
                recovery_behaviors_.push_back(behavior);
            }
            catch(pluginlib::PluginlibException& ex){
                ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                return false;
            }
            }
        }
        else{
            ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list.getType());
            return false;
        }
        }
        else{
        //if no recovery_behaviors are specified, we'll just load the defaults
        return false;
        }

        //if we've made it here... we've constructed a recovery behavior list successfully
        return true;
    }

    //we'll load our default recovery behaviors here
    void MoveBase::loadDefaultRecoveryBehaviors(){
        recovery_behaviors_.clear();
        try{
        //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
        ros::NodeHandle n("~");
        n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
        n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

        //first, we'll load a recovery behavior to clear the costmap
        boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
        cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("conservative_reset");
        recovery_behaviors_.push_back(cons_clear);

        //next, we'll load a recovery behavior to rotate in place
        boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
        if(clearing_rotation_allowed_){
            rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("rotate_recovery");
            recovery_behaviors_.push_back(rotate);
        }

        //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
        boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
        ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("aggressive_reset");
        recovery_behaviors_.push_back(ags_clear);

        //we'll rotate in-place one more time
        if(clearing_rotation_allowed_){
            recovery_behaviors_.push_back(rotate);
            recovery_behavior_names_.push_back("rotate_recovery");
        }
        }
        catch(pluginlib::PluginlibException& ex){
        ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }

    void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        if(!setup_)
        {
        last_config_ = config;
        default_config_ = config;
        setup_ = true;
        return;
        }

        if(config.restore_defaults) {
        config = default_config_;
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        }

        if(planner_frequency_ != config.planner_frequency)
        {
        planner_frequency_ = config.planner_frequency;
        p_freq_change_ = true;
        }

        if(controller_frequency_ != config.controller_frequency)
        {
        controller_frequency_ = config.controller_frequency;
        c_freq_change_ = true;
        }

        planner_patience_ = config.planner_patience;
        controller_patience_ = config.controller_patience;
        max_planning_retries_ = config.max_planning_retries;
        conservative_reset_dist_ = config.conservative_reset_dist;

        recovery_behavior_enabled_ = config.recovery_behavior_enabled;
        clearing_rotation_allowed_ = config.clearing_rotation_allowed;
        shutdown_costmaps_ = config.shutdown_costmaps;

        oscillation_timeout_ = config.oscillation_timeout;
        oscillation_distance_ = config.oscillation_distance;
        if(config.base_global_planner != last_config_.base_global_planner) {
        boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
        //initialize the global planner
        ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
        try {
            planner_ = bgp_loader_.createInstance(config.base_global_planner);

            // wait for the current planner to finish planning
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

            // Clean up before initializing the new planner
            planner_plan_->clear();
            latest_plan_->clear();
            controller_plan_->clear();
            resetState();
            planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

            lock.unlock();
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                    containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
            planner_ = old_planner;
            config.base_global_planner = last_config_.base_global_planner;
        }
        }

        if(config.base_local_planner != last_config_.base_local_planner){
        boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
        //create a local planner
        try {
            tc_ = blp_loader_.createInstance(config.base_local_planner);
            // Clean up before initializing the new planner
            planner_plan_->clear();
            latest_plan_->clear();
            controller_plan_->clear();
            resetState();
            tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                    containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
            tc_ = old_planner;
            config.base_local_planner = last_config_.base_local_planner;
        }
        }

        make_plan_clear_costmap_ = config.make_plan_clear_costmap;
        make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

        last_config_ = config;
    }


}
