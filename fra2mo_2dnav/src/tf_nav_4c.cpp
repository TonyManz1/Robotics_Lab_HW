#include "../include/tf_nav.h"

// Global variables Aruco pose (4b point)
std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false;


TF_NAV::TF_NAV() {

    _position_aruco_pub = _nh.advertise<geometry_msgs::PoseStamped>( "aruco_frame/pose", 1 );
    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );

    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;

    // Define pose of four goals (2b point)
    _goal1_pos << 0.0, 0.0, 0.0;
    _goal1_or << 0.0, 0.0, 0.0, 1.0;
 
    _goal2_pos << 0.0, 0.0, 0.0;
    _goal2_or << 0.0, 0.0, 0.0, 1.0;
 
    _goal3_pos << 0.0, 0.0, 0.0;
    _goal3_or << 0.0, 0.0, 0.0, 1.0;
 
    _goal4_pos << 0.0, 0.0, 0.0;
    _goal4_or << 0.0, 0.0, 0.0, 1.0;

    // Define pose of four goals in order to map all the environment (3a point)
    _goal5_pos << 0.0, 0.0, 0.0;
    _goal5_or << 0.0, 0.0, 0.0, 1.0;
 
    _goal6_pos << 0.0, 0.0, 0.0;
    _goal6_or << 0.0, 0.0, 0.0, 1.0;
 
    _goal7_pos << 0.0, 0.0, 0.0;
    _goal7_or << 0.0, 0.0, 0.0, 1.0;
 
    _goal8_pos << 0.0, 0.0, 0.0;
    _goal8_or << 0.0, 0.0, 0.0, 1.0;    

    // Define pose of goal in order to send the mobile robot in front of the Aruco marker (4b point)
    _goal4b_pos << 0.0, 0.0, 0.0;
    _goal4b_or << 0.0, 0.0, 0.0, 1.0;
    // Define pose of goal in order to send the mobile robot at the intermediate point before to move in front of the Aruco marker (4b point)
    _goal4b_1_pos << 0.0, 0.0, 0.0;
    _goal4b_1_or << 0.0, 0.0, 0.0, 1.0;

    _home_pos << -18.0, 2.0, 0.0;
}

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);

}


// poseCallback function for 4c point (we got it from the website adviced in the pdf)
void poseCallback(const geometry_msgs::PoseStamped & msg) {
    static tf::TransformBroadcaster br;
tf::Transform transform;
transform.setOrigin( tf::Vector3(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z));
tf::Quaternion q(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_frame"));
}

void TF_NAV::tf_listener_aruco() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() ) {
        try {
            listener.waitForTransform( "map", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "camera_depth_optical_frame", ros::Time(0), transform );
        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _camera_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _camera_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
    
        if(aruco_pose_available) {
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));
            KDL::Frame map_T_cam(KDL::Rotation::Quaternion(_camera_or[1], _camera_or[2], _camera_or[3], _camera_or[0]), KDL::Vector(_camera_pos[0], _camera_pos[1], _camera_pos[2]));
            KDL::Frame map_T_object(KDL::Rotation::Quaternion(_aruco_or[1], _aruco_or[2], _aruco_or[3], _aruco_or[0]), KDL::Vector(_aruco_pos[0], _aruco_pos[1], _aruco_pos[2]));

            map_T_object.p = map_T_cam.p + map_T_cam.M*cam_T_object.p;
            _aruco_pos = toEigen(map_T_object.p);

            map_T_object.M = map_T_cam.M*cam_T_object.M;
            double x,y,z,w;
            map_T_object.M.GetQuaternion(x,y,z,w);
            _aruco_or << x, y, z, w;
            
            std::cout<<"Aruco position wrt map frame:"<<std::endl<< _aruco_pos <<std::endl;
            //std::cout<<"Aruco quaternion wrt map frame:"<< _aruco_or <<std::endl;
            std::cout<<"Camera position wrt map frame:"<<std::endl<< toEigen(map_T_cam.p) <<std::endl;
            //std::cout<<"Aruco position wrt camera frame:"<< toEigen(cam_T_object.p) <<std::endl;
        }
        position_aruco_pub();
        r.sleep();
    }

}


void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );

        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }

}


void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);

}


void TF_NAV::position_aruco_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _aruco_pos[0];
    pose.pose.position.y = _aruco_pos[1];
    pose.pose.position.z = _aruco_pos[2];

    pose.pose.orientation.w = _aruco_or[0];
    pose.pose.orientation.x = _aruco_or[1];
    pose.pose.orientation.y = _aruco_or[2];
    pose.pose.orientation.z = _aruco_or[3];

    _position_aruco_pub.publish(pose);
}

// Goal listener functions (2b point)

void TF_NAV::goal_listener_1() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal1", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal1", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal1_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal1_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        // // Debug Print
        // ROS_INFO("Goal_1 Position: %f %f %f", _goal1_pos[0], _goal1_pos[1], _goal1_pos[2]);
        // ROS_INFO("Goal_1 Orientation: %f %f %f %f", _goal1_or[0], _goal1_or[1], _goal1_or[2], _goal1_or[3]);
 
        r.sleep();
    }    
}
 
void TF_NAV::goal_listener_2() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal2", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal2", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal2_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal2_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
 
        // // Debug Print
        // ROS_INFO("Goal_2 Position: %f %f %f", _goal2_pos[0], _goal2_pos[1], _goal2_pos[2]);
        // ROS_INFO("Goal_2 Orientation: %f %f %f %f", _goal2_or[0], _goal2_or[1], _goal2_or[2], _goal2_or[3]);

        r.sleep();
    }    
}
 
void TF_NAV::goal_listener_3() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal3", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal3", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal3_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal3_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        // // Debug Print
        // ROS_INFO("Goal_3 Position: %f %f %f", _goal3_pos[0], _goal3_pos[1], _goal3_pos[2]);
        // ROS_INFO("Goal_3 Orientation: %f %f %f %f", _goal3_or[0], _goal3_or[1], _goal3_or[2], _goal3_or[3]); 

        r.sleep();
    }    
}
 
void TF_NAV::goal_listener_4() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal4", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal4", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal4_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal4_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        // // Debug Print
        // ROS_INFO("Goal_4 Position: %f %f %f", _goal4_pos[0], _goal4_pos[1], _goal4_pos[2]);
        // ROS_INFO("Goal_4 Orientation: %f %f %f %f", _goal4_or[0], _goal4_or[1], _goal4_or[2], _goal4_or[3]);        

        r.sleep();
    }    
}

// Goal listener functions for 3a point

void TF_NAV::goal_listener_5() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal5", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal5", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal5_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal5_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        r.sleep();
    }    
}
 
void TF_NAV::goal_listener_6() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal6", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal6", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal6_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal6_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        r.sleep();
    }    
}
 
void TF_NAV::goal_listener_7() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal7", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal7", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal7_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal7_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        r.sleep();
    }    
}
 
void TF_NAV::goal_listener_8() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal8", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal8", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal8_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal8_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();    

        r.sleep();
    }    
}

void TF_NAV::goal_listener_4b() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal4b", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal4b", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal4b_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal4b_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();    

        r.sleep();
    }    
}

void TF_NAV::goal_listener_4b_1() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal4b_1", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal4b_1", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal4b_1_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal4b_1_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();    

        r.sleep();
    }    
}

// Modify send_goal function in order to follow the sequence of the goals of the 2c point

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    int a_cmd;
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() )
    {
         std::cout<<"                                                                                           "<<std::endl;
            std::cout<<"\033[94m                                   ______                                 \033[1;94m"<<std::endl;
            std::cout<<"\033[94m                                  |      |                                \033[1;94m"<<std::endl;
            std::cout<<"\033[94m                                 | (\033[1;94m\033[1;31m-\033[1;94m\033[1;94m)(\033[1;94m\033[1;31m-\033[1;94m\033[1;94m) |  ____________________________________________________\033[1;94m" << std::endl;
            std::cout<<"\033[94m                                 |   _    | |                                                   / \033[1;94m"<<std::endl;
            std::cout<<"\033[94m                                 |  |_|   | | \033[38;2;255;165;0mInsert your choice. Select:                     \033[1;94m \033[1;94m/ \033[1;94m\033[1;94m\033[1;94m" << std::endl;
            std::cout<<"\033[94m                                  |______|  |_________________________________________________/   \033[1;94m"<<std::endl;
            std::cout<<"\033[94m                       _____________| |____________                       \033[1;94m"<<std::endl;
            std::cout<<"\033[94m    __________________|____________________________|___________________   \033[1;94m"<<std::endl;
            std::cout<<"\033[94m   |                                                                   |  \033[1;94m"<<std::endl;
            std::cout<<"\033[38;2;255;165;0m   \033[1;94m|\033[38;2;255;165;0m                                                                   \033[1;94m|\033[1;94m" << std::endl;
            std::cout<<"\033[94m   |___________________________________________________________________|  \033[1;94m"<<std::endl;
            std::cout<<"\033[94m   |                                                                   |  \033[1;94m"<<std::endl;
            std::cout<<"\033[38;2;255;165;0m   \033[1;94m|\033[38;2;255;165;0m  1 -> Send goal from TF                                           \033[1;94m|\033[1;94m" << std::endl;                                        
            std::cout<<"\033[94m   |___________________________________________________________________|  \033[1;94m"<<std::endl; 
            std::cout<<"\033[94m   |                                                                   |  \033[1;94m"<<std::endl; 
            std::cout<<"\033[38;2;255;165;0m   \033[1;94m|\033[38;2;255;165;0m  2 -> Send home position goal                                     \033[1;94m|\033[1;94m" << std::endl;                                   
            std::cout<<"\033[94m   |___________________________________________________________________|  \033[1;94m"<<std::endl; 
            std::cout<<"\033[94m   |                                                                   |  \033[1;94m"<<std::endl; 
            std::cout<<"\033[38;2;255;165;0m   \033[1;94m|\033[38;2;255;165;0m  3 -> Send position goal in front of the Aruco marker             \033[1;94m|\033[1;94m" << std::endl;                                       
            std::cout<<"\033[94m   |___________________________________________________________________|  \033[1;94m"<<std::endl; 
            std::cout<<"\033[94m                  |____________________________________|                  \033[1;94m"<<std::endl; 
            std::cout<<"\033[94m                      ||                          ||                      \033[1;94m"<<std::endl; 
            std::cout<<"\033[94m                      ||                          ||                      \033[1;94m"<<std::endl; 
            std::cout<<"\033[94m" << "....................._||_........................_||_....................." << "\033[1;94m" << std::endl;
            std::cout<<"\033[38;2;255;165;0m\nChoice:\t\033[1;94m";
            
        std::cin>>cmd; 


        if ( cmd == 1 ) {
           MoveBaseClient ac3("move_base", true);

            while(!ac3.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // First goal of the sequence is Goal_3
            goal.target_pose.pose.position.x = _goal3_pos[0];
            goal.target_pose.pose.position.y = _goal3_pos[1];
            goal.target_pose.pose.position.z = _goal3_pos[2];

            goal.target_pose.pose.orientation.w = _goal3_or[0];
            goal.target_pose.pose.orientation.x = _goal3_or[1];
            goal.target_pose.pose.orientation.y = _goal3_or[2];
            goal.target_pose.pose.orientation.z = _goal3_or[3];

            ROS_INFO("Sending goal 3");
            ac3.sendGoal(goal);

            ac3.waitForResult();

            if(ac3.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_3");

            else
                ROS_INFO("The base failed to move for some reason");

            MoveBaseClient ac4("move_base", true);
            while(!ac4.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Second goal of the sequence is Goal_4
            goal.target_pose.pose.position.x = _goal4_pos[0];
            goal.target_pose.pose.position.y = _goal4_pos[1];
            goal.target_pose.pose.position.z = _goal4_pos[2];

            goal.target_pose.pose.orientation.w = _goal4_or[0];
            goal.target_pose.pose.orientation.x = _goal4_or[1];
            goal.target_pose.pose.orientation.y = _goal4_or[2];
            goal.target_pose.pose.orientation.z = _goal4_or[3];

            ROS_INFO("Sending goal 4");
            ac4.sendGoal(goal);

            ac4.waitForResult();

            if(ac4.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_4");
            
            else
                ROS_INFO("The base failed to move for some reason");

            MoveBaseClient ac2("move_base", true);
            while(!ac2.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Third goal of the sequence is Goal_2
            goal.target_pose.pose.position.x = _goal2_pos[0];
            goal.target_pose.pose.position.y = _goal2_pos[1];
            goal.target_pose.pose.position.z = _goal2_pos[2];

            goal.target_pose.pose.orientation.w = _goal2_or[0];
            goal.target_pose.pose.orientation.x = _goal2_or[1];
            goal.target_pose.pose.orientation.y = _goal2_or[2];
            goal.target_pose.pose.orientation.z = _goal2_or[3];

            ROS_INFO("Sending goal 2");
            ac2.sendGoal(goal);

            ac2.waitForResult();

            if(ac2.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_2");
            
            else
                ROS_INFO("The base failed to move for some reason");

            MoveBaseClient ac1("move_base", true);
            while(!ac1.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Fourth goal of the sequence is Goal_1
            goal.target_pose.pose.position.x = _goal1_pos[0];
            goal.target_pose.pose.position.y = _goal1_pos[1];
            goal.target_pose.pose.position.z = _goal1_pos[2];

            goal.target_pose.pose.orientation.w = _goal1_or[0];
            goal.target_pose.pose.orientation.x = _goal1_or[1];
            goal.target_pose.pose.orientation.y = _goal1_or[2];
            goal.target_pose.pose.orientation.z = _goal1_or[3];

            ROS_INFO("Sending goal 1");
            ac1.sendGoal(goal);

            ac1.waitForResult();

            if(ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_1");
            
            else
                ROS_INFO("The base failed to move for some reason");

            MoveBaseClient ac5("move_base", true);
            while(!ac5.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Fifth goal of the sequence is Goal_5 to map all environment (3a point)
            goal.target_pose.pose.position.x = _goal5_pos[0];
            goal.target_pose.pose.position.y = _goal5_pos[1];
            goal.target_pose.pose.position.z = _goal5_pos[2];

            goal.target_pose.pose.orientation.w = _goal5_or[0];
            goal.target_pose.pose.orientation.x = _goal5_or[1];
            goal.target_pose.pose.orientation.y = _goal5_or[2];
            goal.target_pose.pose.orientation.z = _goal5_or[3];

            ROS_INFO("Sending goal 5");
            ac5.sendGoal(goal);

            ac5.waitForResult();

            if(ac5.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_5");
            
            else
                ROS_INFO("The base failed to move for some reason");

            MoveBaseClient ac6("move_base", true);
            while(!ac6.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Sixth goal of the sequence is Goal_6 to map all environment (3a point)
            goal.target_pose.pose.position.x = _goal6_pos[0];
            goal.target_pose.pose.position.y = _goal6_pos[1];
            goal.target_pose.pose.position.z = _goal6_pos[2];

            goal.target_pose.pose.orientation.w = _goal6_or[0];
            goal.target_pose.pose.orientation.x = _goal6_or[1];
            goal.target_pose.pose.orientation.y = _goal6_or[2];
            goal.target_pose.pose.orientation.z = _goal6_or[3];

            ROS_INFO("Sending goal 6");
            ac6.sendGoal(goal);

            ac6.waitForResult();

            if(ac6.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_6");
            
            else
                ROS_INFO("The base failed to move for some reason");

            MoveBaseClient ac7("move_base", true);
            while(!ac7.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Seventh goal of the sequence is Goal_7 to map all environment (3a point)
            goal.target_pose.pose.position.x = _goal7_pos[0];
            goal.target_pose.pose.position.y = _goal7_pos[1];
            goal.target_pose.pose.position.z = _goal7_pos[2];

            goal.target_pose.pose.orientation.w = _goal7_or[0];
            goal.target_pose.pose.orientation.x = _goal7_or[1];
            goal.target_pose.pose.orientation.y = _goal7_or[2];
            goal.target_pose.pose.orientation.z = _goal7_or[3];

            ROS_INFO("Sending goal 7");
            ac7.sendGoal(goal);

            ac7.waitForResult();

            if(ac7.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_7");
            
            else
                ROS_INFO("The base failed to move for some reason");

            MoveBaseClient ac8("move_base", true);
            while(!ac8.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Eighth goal of the sequence is Goal_8 to map all environment (3a point)
            goal.target_pose.pose.position.x = _goal8_pos[0];
            goal.target_pose.pose.position.y = _goal8_pos[1];
            goal.target_pose.pose.position.z = _goal8_pos[2];

            goal.target_pose.pose.orientation.w = _goal8_or[0];
            goal.target_pose.pose.orientation.x = _goal8_or[1];
            goal.target_pose.pose.orientation.y = _goal8_or[2];
            goal.target_pose.pose.orientation.z = _goal8_or[3];

            ROS_INFO("Sending goal 8");
            ac8.sendGoal(goal);

            ac8.waitForResult();

            if(ac8.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_8");
            
            else
                ROS_INFO("The base failed to move for some reason");
        }
            else if(cmd != 2 && cmd != 3) {
            ROS_INFO("Wrong input!");
        }

        
        if ( cmd == 2 ) {

            MoveBaseClient ach("move_base", true);
            while(!ach.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_INFO("Sending HOME position as goal");
            ach.sendGoal(goal);

            ach.waitForResult();

            if(ach.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the HOME position");
            
            else
                ROS_INFO("The base failed to move for some reason");
        }
         else if(cmd != 1 && cmd != 3 ){
            ROS_INFO("Wrong input!");
        }
    

           if ( cmd == 3 ) {

            MoveBaseClient ac41("move_base", true);
            while(!ac41.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // 4b_1 goal is an intermediate point before mobile robot moves in front of Aruco 
            goal.target_pose.pose.position.x = _goal4b_1_pos[0];
            goal.target_pose.pose.position.y = _goal4b_1_pos[1];
            goal.target_pose.pose.position.z = _goal4b_1_pos[2];

            goal.target_pose.pose.orientation.w = _goal4b_1_or[0];
            goal.target_pose.pose.orientation.x = _goal4b_1_or[1];
            goal.target_pose.pose.orientation.y = _goal4b_1_or[2];
            goal.target_pose.pose.orientation.z = _goal4b_1_or[3];

            ROS_INFO("Sending goal 4b_1");
            ac41.sendGoal(goal);

            ac41.waitForResult();

            if(ac41.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot is in the pose of the Goal_4b_1");
            
            else
                ROS_INFO("The base failed to move for some reason");
            
            MoveBaseClient ac4b("move_base", true);
            while(!ac4b.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = _goal4b_pos[0];
            goal.target_pose.pose.position.y = _goal4b_pos[1];
            goal.target_pose.pose.position.z = _goal4b_pos[2];
 
            goal.target_pose.pose.orientation.w = _goal4b_or[0];
            goal.target_pose.pose.orientation.x = _goal4b_or[1];
            goal.target_pose.pose.orientation.y = _goal4b_or[2];
            goal.target_pose.pose.orientation.z = _goal4b_or[3];
 
            ROS_INFO("Sending goal 4b");
            ac4b.sendGoal(goal);
 
            ac4b.waitForResult();
 
            if(ac4b.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The mobile robot arrived in the TF goal 4b");
            else
                ROS_INFO("The base failed to move for some reason");
 

            // When the mobile robot arrived in the goal4b it moves at this pose (x = x_m + 1 and y = y_m) when the user insert the command "4"

            std::cout<<"\nInsert 4 to send Aruco goal"<<std::endl;
            std::cin>>a_cmd;
            if ( a_cmd == 4) {
                MoveBaseClient ac_a("move_base", true);
                while(!ac_a.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
                }
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                
                goal.target_pose.pose.position.x = _aruco_pos[0] + 1;
                goal.target_pose.pose.position.y = _aruco_pos[1];
                goal.target_pose.pose.position.z = 0.0;
/*
                goal.target_pose.pose.orientation.w = 0.0;
                goal.target_pose.pose.orientation.x = 0.0;
                goal.target_pose.pose.orientation.y = 0.0;
                goal.target_pose.pose.orientation.z = 1.0;
*/               
                KDL::Rotation Ar_rot=KDL::Rotation::Quaternion(_aruco_or[1], _aruco_or[2], _aruco_or[3], _aruco_or[0]);
                KDL::Rotation Rb_des=Ar_rot*KDL::Rotation::RotX(-1.5708)*KDL::Rotation::RotZ(1.5708);
                double R,P,Y,x,y,z,w;
                Rb_des.GetRPY(R,P,Y);
                Rb_des=KDL::Rotation::RPY(0,0,Y);
                Rb_des.GetQuaternion(x,y,z,w);
                
                goal.target_pose.pose.orientation.w = w;
                goal.target_pose.pose.orientation.x = x;
                goal.target_pose.pose.orientation.y = y;
                goal.target_pose.pose.orientation.z = z;

                ROS_INFO("Sending goal aruco");
                ac_a.sendGoal(goal);
    
                ac_a.waitForResult();
    
                if(ac_a.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("The mobile robot arrived in the TF aruco goal");
                else
                    ROS_INFO("The base failed to move for some reason");

            }
            else {
                ROS_INFO("Wrong input!");
            }
            
           }
           else if (cmd != 1 && cmd != 2){
                ROS_INFO("Wrong input!");
            }

        r.sleep();
    }
    
}

void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal1_t( &TF_NAV::goal_listener_1, this );
    boost::thread tf_listener_goal2_t( &TF_NAV::goal_listener_2, this );
    boost::thread tf_listener_goal3_t( &TF_NAV::goal_listener_3, this );
    boost::thread tf_listener_goal4_t( &TF_NAV::goal_listener_4, this );
    boost::thread tf_listener_goal5_t( &TF_NAV::goal_listener_5, this );
    boost::thread tf_listener_goal6_t( &TF_NAV::goal_listener_6, this );
    boost::thread tf_listener_goal7_t( &TF_NAV::goal_listener_7, this );
    boost::thread tf_listener_goal8_t( &TF_NAV::goal_listener_8, this );
    boost::thread tf_listener_goal4b_t( &TF_NAV::goal_listener_4b, this );
    boost::thread tf_listener_goal4b_1_t( &TF_NAV::goal_listener_4b_1, this );
    boost::thread tf_listener_aruco_t( &TF_NAV::tf_listener_aruco, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}


int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    ros::NodeHandle n;
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
    ros::Subscriber aruco_pose_sub_broadc = n.subscribe("/aruco_frame/pose", 1, poseCallback);
    TF_NAV tfnav;
    tfnav.run();

    return 0;
}
