
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h" 
#include "math.h"
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include "math.h"

//#include "amr_navigation/bug2.h"

using namespace std;


class Bug2{
public:
    // Encapsulation
    geometry_msgs::PoseStamped getQStart();
    void setQStart(geometry_msgs::PoseStamped qStart);
    geometry_msgs::PoseStamped getQGoal();
    void setQGoal(geometry_msgs::PoseStamped qGoal);
    vector<geometry_msgs::PoseStamped> getQL();
    void setQL(vector<geometry_msgs::PoseStamped> qL);
    vector<geometry_msgs::PoseStamped> getQH();
    void setQH(vector<geometry_msgs::PoseStamped> qH);
    geometry_msgs::PoseStamped getX();
    void setX(geometry_msgs::PoseStamped x);

    // Constructor
    Bug2();
    ~Bug2();

    // Methods
    void Initialization(Robot robot, geometry_msgs::PoseStamped qGoal);
    geometry_msgs::Twist goAlongMLine(Robot robot, geometry_msgs::PoseStamped qL, geometry_msgs::PoseStamped qStart);
    bool doesObstacleEncountered(Robot robot);
    bool isGoalReached(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped goal);
    geometry_msgs::Twist followBoundary(Robot robot, int waySelection);
    int waySelection(Robot robot);
    vector<geometry_msgs::PoseStamped> mLinePoints(geometry_msgs::PoseStamped qStart, geometry_msgs::PoseStamped qGoal);
    bool mLineReencountered(geometry_msgs::PoseStamped x, vector<geometry_msgs::PoseStamped> points);
    bool isXEqualToQHi(geometry_msgs::PoseStamped x, geometry_msgs::PoseStamped qHi);
    bool distanceComparition(geometry_msgs::PoseStamped x, geometry_msgs::PoseStamped qHi, geometry_msgs::PoseStamped qGoal);

private:
    geometry_msgs::PoseStamped qStart_;      // start pose 
    geometry_msgs::PoseStamped qGoal_;       // goal pose
    vector<geometry_msgs::PoseStamped> qL_;  // leaves pose
    vector<geometry_msgs::PoseStamped> qH_;  // heads pose
    geometry_msgs::PoseStamped x_;           // current pose
};

// Encapsulation
    geometry_msgs::PoseStamped Bug2::getQStart(){
        return qStart_;
    }
    void Bug2::setQStart(geometry_msgs::PoseStamped qStart){
        qStart_ = qStart;
    }
    geometry_msgs::PoseStamped Bug2::getQGoal(){
        return qGoal_;
    }
    void Bug2::setQGoal(geometry_msgs::PoseStamped qGoal){
        qGoal_ = qGoal;
    }
    vector<geometry_msgs::PoseStamped> Bug2::getQL(){
        return qL_;
    }
    void Bug2::setQL(vector<geometry_msgs::PoseStamped> qL){
        qL_ = qL;
    }
    vector<geometry_msgs::PoseStamped> Bug2::getQH(){
        return qH_;
    }
    void Bug2::setQH(vector<geometry_msgs::PoseStamped> qH){
        qH_ = qH;
    }
    geometry_msgs::PoseStamped Bug2::getX(){
        return x_;
    }
    void Bug2::setX(geometry_msgs::PoseStamped x){
        x_ = x;
    }

    Bug2::Bug2(){
        ROS_INFO("bug2 instance created as zero");
    }
    Bug2::~Bug2(){

    }

// Methods

/**
 * @brief initialization:
 *        current pose: x_, starrt pose: qStart_, first leave point: qL_
 *        first head point is null, goal come from parameter
 * 
 * @param robot robot instance; ex: Robot robot1 in main
 * @param qGoal goal pose of bug2 algorithm
 */
    void Bug2::Initialization(Robot robot, geometry_msgs::PoseStamped qGoal){
        // x_: current pose
            // x_ coordinate:
        x_.pose.position.x = robot.getX();
        x_.pose.position.y = robot.getY();
        x_.pose.position.z = 0;
            // Euler to quaternion:
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0,0,robot.getTheta());
        myQuaternion=myQuaternion.normalize();
            //x_ orientation:
        x_.pose.orientation.x = myQuaternion.getX();
        x_.pose.orientation.x = myQuaternion.getY();
        x_.pose.orientation.x = myQuaternion.getZ();
        x_.pose.orientation.x = myQuaternion.getW();

        // qStart_: start pose:
        qStart_ = x_;

        // qL_: qL(i), i is 0 in pseudo code, is leave point, equlas to start pose:
        qL_.push_back(qStart_);

        // qH_: qH(0) is head point, equal to null.
        geometry_msgs::PoseStamped qH;
        qH.pose.position.x = NULL;
        qH.pose.position.y = NULL;
        qH.pose.position.z = NULL;

        qH.pose.orientation.x = NULL;
        qH.pose.orientation.y = NULL;
        qH.pose.orientation.z = NULL;
        qH.pose.orientation.w = NULL;

        qH_.push_back(qH);

        // qGoal:
        qGoal_ = qGoal;
    }

    /**
     * @brief in pseudo code: from qL(i-1) move toward qgoal along the m-line.
     *        robot go from qL to other head(obstacle encountered) pose.
     *        Robot move along M-line with PID
     * 
     * @param robot robot instance; ex: Robot robot1 in main
     * @param qL leave point: geometry_msgs::PoseStamped type
     * @param qGoal goal or heading point: geometry_msgs::PoseStamped type
     * @return geometry_msgs::Twist velocity must published cmd_vel (velocity) topic
     */
    geometry_msgs::Twist Bug2::goAlongMLine(Robot robot, geometry_msgs::PoseStamped qL, geometry_msgs::PoseStamped qGoal){
        geometry_msgs::Twist velocity;
        double theta = robot.getTheta();
        if(theta < 0)
            theta = 6.28 + theta;
        cout << "theta: " << theta << endl;
        
        double goal_yaw = 0;    // initialized as zero, this is not hard-coding

        double start_x = qL.pose.position.x;
        double start_y = qL.pose.position.y;
        double goal_x = qGoal.pose.position.x;
        double goal_y = qGoal.pose.position.y;

        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;

        goal_yaw = atan2(diff_y, diff_x);
        if(goal_yaw < 0)
            goal_yaw = 6.28 + goal_yaw;
        cout << "goal_yaw: " << goal_yaw << endl;
        //PID CONTROL VARIABLES//////////////
        float angle = goal_yaw;
        float th = theta;
        float zw = 0;
        float period = 0.1;   // 100mS
        float error = 0;      float error1 = 0;     float error2 = 0;     float error3 = 0;
        float th1 = 0;        float zw_old = 0;
        float Kp = 0.2;        //0.05
        float Ki = 1; //6     //0.1
        float Kd = 0.7;       //0.01
        int zwx = 0;

        error = angle - th;

        zw = Kp*error + Ki*period*(angle-(th+th1)/2.0) +
        (Kd/(6*period))*((error-error3)+3*(error1-error2)) + zw_old;
        
        th1 = th;
        error3 = error2;
        error2 = error1;
        error1 = error;
        zw_old = zw;

        if(zw>2)
            zw = 2;

        velocity.angular.z = zw;
        velocity.linear.x = 0.2; 

/*
        cout << endl;
        cout << "error: " << error << endl;
        cout << "zw: " << zw << endl;
        cout << "velocity.linear.x: " << velocity.linear.x << endl;
        cout << "theta: " << theta << endl;
        cout << "------------------" << endl;
*/
        return velocity;
    }

/**
 * @brief 
 * 
 * @param robot robot instance; ex: Robot robot1 in main
 * @return true robot encounter with obstacle for 0.3m and few meter
 * @return false robot does not see obstacle for 0.3m and few meter, unimpeded
 */
    bool Bug2::doesObstacleEncountered(Robot robot){
        // cout <<"obstacle distance: " << robot.getCenterDistance() << endl;
        if(robot.getCenterDistance() < 0.001){
            ROS_WARN("Gazebo laser data not initialized");
            return false;
        }
        if(robot.getCenterDistance() < 0.3 ){ // center sensor value
            
           // ROS_INFO("Obstacle Encountered");
           // cout <<"obstacle distance: " << robot.getCenterDistance() << endl;
            
            return true;
        }
        else{
            return false;
        }
    }

/**
 * @brief if robot reach goel pose, this function return true
 * 
 * @param current Bug2 class x_ field, ex: Bug2 bug2; bug2.getX()
 * @param goal Bug2 class qGoal_ field, ex: Bug2 bug2; bug2.getQGoal()
 * @return true goal reached 
 * @return false goal doesn't reached
 */
    bool Bug2::isGoalReached(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped goal){
        double diff_x = goal.pose.position.x - current.pose.position.x;
        double diff_y = goal.pose.position.y - current.pose.position.y;
        double abs_diff_x = abs(diff_x);
        double abs_diff_y = abs(diff_y);

        if(abs_diff_x < 0.3 && abs_diff_y < 0.3){
            ROS_INFO("goal is reached!");
            return true;
        }
        else{
            return false;
        }
    }

/**
 * @brief PID for wall following process
 * 
 * @param laserData left or right sensor data, ex: robot.getRightDistance()
 * @return float PID angular velocity 
 */
    float wallFollowPID(double laserData){
        //PID CONTROL VARIABLES//////////////
        if(laserData > 3)
            laserData = 3;

        float desiredDistance = 0.6;    // 0.6 meter
        float sensorDistance = laserData;
        float zw = 0;
        float period = 0.1;   // 100mS
        float error = 0;      float error1 = 0;     float error2 = 0;     float error3 = 0;
        float sensorDistance1 = 0;        float zw_old = 0;
        float Kp = 0.05;        //0.05
        float Ki = 1.7; //6     //3
        float Kd = 1.3;       //2.7
        int zwx = 0;

        error = desiredDistance - sensorDistance;

        zw = Kp*error + Ki*period*(desiredDistance-(sensorDistance+sensorDistance1)/2.0) +
        (Kd/(6*period))*((error-error3)+3*(error1-error2)) + zw_old;
        
        sensorDistance1 = sensorDistance;
        error3 = error2;
        error2 = error1;
        error1 = error;
        zw_old = zw;

        if(zw > 1)
            zw = 1;

        // ROS_INFO("angular vel: %2.2f",zw);
        // ROS_INFO("laserData: %2.2f",laserData);
        return zw;
    }

/**
 * @brief Robot follow wall from qH(i) to qL(i); in pseudecode
 *        This function produce velocity for wall following as PID.
 * 
 * @param robot robot laser data is used for function
 * @param waySelection 1: left, 2: right
 * @return geometry_msgs::Twist velocity must published cmd_vel topic.
 */
    geometry_msgs::Twist Bug2::followBoundary(Robot robot, int waySelection){
        geometry_msgs::Twist velocity;
        // 0.6: 60cm at laser distance
        if(waySelection == 1 ){
            // left selection
            velocity.linear.x = 0.2;
            velocity.angular.z = -1*wallFollowPID(robot.getLeftDistance());
        }
        else if(waySelection == 2){
            // right selection
            velocity.linear.x = 0.2;
            velocity.angular.z = wallFollowPID(robot.getRightDistance());
        }
        else{
            ROS_ERROR("waySelection must be 1, if left; waySelection must be 1, if right: waySelection: %i ",waySelection);
        }

        return velocity;
    }

/**
 * @brief followBoundary function require waySelection. left or right sensor value proccessed
 *        for wall following.
 * 
 * @param robot robot left and right sensor data used for data
 * @return int if return 1: left selection, if return 2: right selection
 */
    int Bug2::waySelection(Robot robot){
        if(robot.getLeftDistance() <= 4 && (robot.getLeftDistance() <= robot.getRightDistance())){
            // left wall select
            return 1;
        }
        else if(robot.getRightDistance() <= 4 && (robot.getRightDistance() <= robot.getRightDistance())){
            // right wall select
            return 2;
        }
        else{
            ROS_ERROR("no wall detected, left: %2.2lf, right: %2.2lf",
                       robot.getLeftDistance(),robot.getRightDistance());
        }
    }

/**
 * @brief in follow boundary condition, m line re-encountered needed.
 *        so, points on M-line must be known. This give points on M-line.

 * @param qStart bug2.getQStart(): start point
 * @param qGoal bug2.getQGoal(): goal point
 * @return type vector<geometry_msgs::PoseStamped>: vector of points on M-line
 */
    vector<geometry_msgs::PoseStamped> Bug2::mLinePoints(geometry_msgs::PoseStamped qStart, geometry_msgs::PoseStamped qGoal){
        vector<geometry_msgs::PoseStamped> mLinePoints; // poses on M-line 
        geometry_msgs::PoseStamped point;               // adding point in M-line
        double start_x = qStart.pose.position.x;
        double start_y = qStart.pose.position.y;
        double goal_x = qGoal.pose.position.x;
        double goal_y = qGoal.pose.position.y;
        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;
        double abs_diff_x = abs(diff_x);
        double abs_diff_y = abs(diff_y);
        int iterator = 0;                           // iterator: count of points

        if(abs_diff_x > abs_diff_y){
            iterator = abs_diff_x * 10;             // 0.1m resolution of between points distance
        }
        else{
            iterator = abs_diff_y * 10;             // 0.1m resolution
        }

        for(int i = 0; i < iterator; i++){
            point.pose.position.x = start_x + diff_x*i/iterator;
            point.pose.position.y = start_y + diff_y*i/iterator;
            mLinePoints.push_back(point);
        }
        
        return mLinePoints;
    }

/**
 * @brief in pseudo code: m-line is re-encountered statement: if m-line is re-encountered
 *        check other parameter in wall following. 
 * 
 * @param x bug2.getX(): current pose
 * @param points mLinePoints(bug2.getQStart(), bug2.getQGoal()): m-line points
 * @return true check other statements,  m-line is re-encountered
 * @return false contiune wall following if, goal doesn't reached,  m-line isn't re-encountered
 */
    bool Bug2::mLineReencountered(geometry_msgs::PoseStamped x, vector<geometry_msgs::PoseStamped> points){
        double current_x = x.pose.position.x;
        double current_y = x.pose.position.y;

        vector<geometry_msgs::PoseStamped>::iterator iter = points.begin();
        while(iter != points.end()){
            double point_x = iter->pose.position.x;
            double point_y = iter->pose.position.y;
            double diff_x = point_x - current_x;
            double diff_y = point_y - current_y;
            double abs_diff_x = abs(diff_x);
            double abs_diff_y = abs(diff_y);

            if(abs_diff_x < 0.3 && abs_diff_y < 0.3){
                return true;    //  m-line is re-encountered
            }
            iter++;
        }
        return false;           //  m-line isn't re-encountered
    }

/**
 * @brief in pseudocode: x is not qHi, if x equal qHi, continue wall following
 *        else go m-line. Note: in bug2 algorithm, use false condition!!!!!
 *        in bug2 algorithm: if(!isXEqualToQHi(bug2.getX(),bug2.getQH().back()))
 * 
 * @param x bug2.getX(): current pose
 * @param qHi bug2.getQH().back() : current head pose
 * @return true continue wall following if other parameter is provided
 * @return false go m-line if other parameter is provided
 */
    bool Bug2::isXEqualToQHi(geometry_msgs::PoseStamped x, geometry_msgs::PoseStamped qHi){
        double current_x = x.pose.position.x;
        double current_y = x.pose.position.y;
        double qHx = qHi.pose.position.x;
        double qHy = qHi.pose.position.y;
        double diff_x = qHx - current_x;
        double diff_y = qHy - current_y;
        double abs_diff_x = abs(diff_x);
        double abs_diff_y = abs(diff_y);

        if(abs_diff_x < 0.3 && abs_diff_y < 0.3){
            return true;    // x equal qHi
        }
        else{
            return false;
        }
    }

/**
 * @brief in pseudo code: d(x, qgoal) < d(qHi, qgoal); if x to goal distance less than
 *        qHi to goal distance, go m-line, else contiune wall following
 * 
 * @param x bug2.getX(): current pose
 * @param qHi bug2.getQH().back() : current head pose
 * @param qGoal bug2.getQGoal() : goal pose
 * @return true go m-line if other parameter is provided
 * @return false contiune wall following if other parameter is provided
 */
    bool Bug2::distanceComparition(geometry_msgs::PoseStamped x, geometry_msgs::PoseStamped qHi, geometry_msgs::PoseStamped qGoal){
        double current_x = x.pose.position.x;
        double current_y = x.pose.position.y;
        double qHx = qHi.pose.position.x;
        double qHy = qHi.pose.position.y;
        double goal_x = qGoal.pose.position.x;
        double goal_y = qGoal.pose.position.y;
        double xToGoal_x = goal_x - current_x;
        double xToGoal_y = goal_y - current_y;
        double qHiToGoal_x = goal_x - qHx;
        double qHiToGoal_y = goal_y - qHy;
        double abs_xToGoal_x = abs(xToGoal_x);
        double abs_xToGoal_y = abs(xToGoal_y);
        double abs_qHiToGoal_x = abs(qHiToGoal_x);
        double abs_qHiToGoal_y = abs(qHiToGoal_y);

        double distance_X_ToGoal = sqrt(abs_xToGoal_x*abs_xToGoal_x + abs_xToGoal_y*abs_xToGoal_y);
        double distance_QH_ToGoal = sqrt(abs_qHiToGoal_x*abs_qHiToGoal_x + abs_qHiToGoal_y*abs_qHiToGoal_y);

        if(distance_X_ToGoal < distance_QH_ToGoal){
            return true;    // d(x, qgoal) < d(qHi, qgoal)
        }
        else{
            return false;   // d(x, qgoal) < d(qHi, qgoal) case is not true
        }
    }
