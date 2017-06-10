#include "../include/follow_ball/find_ball.h"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/tf.h>

#include <wiimote/State.h>

#include "opencv2/opencv.hpp"

///desired radius of the ball in the image
#define DESIRED_R 55
///desired Y-coordinate of the center of the ball, if higher, go backwards (ball at the bottom of the image
#define DESIRED_Y 3/4
///how many steps should be saved for the integral part of the pid controller
#define NUMBER_INT 10
///value for PID controller (using r value of circle to track distance)
#define KP 1.0
///values for PID controller (using r value of circle to track distance)
#define KI 2/NUMBER_INT
///values for PID controller (using r value of circle to track distance)
#define KD 0.5
///value for P controller (using y koordinate to track distance)
#define KPY -1/4

///max angle of camera (add a bit so it follows more than it should)
#define ANGLE_MAX M_PI_4
///orientation controller is quadratic, it trys way harder if the ball is at the edge of the image (OA*xdiff² + OB*xdiff)
#define OA 0.5
///orientation controller is quadratic, it trys way harder if the ball is at the edge of the image (OA*xdiff² + OB*xdiff)
#define OB 1

///range which should stop movement
#define RANGE_MIN 0.12
///stop if near a wall
bool stop = false;
///seq number of goal for autonomous driving
int num = 0;
///corresponce to the middlepoint of the image
double desired_x = 0;
///corresponce to DESIRED_Y
double desired_y = 0;
///Array for the Integral-Part of the PID Controller
std::vector<double> linear_int;
///current index of lienar_int
int linear_int_idx = 0;
///should the images be shown on screen?
bool show = false;
///should it use autnomous driving (send goals) or try to follow directly (send servo commands) !WARNING -> no avoiding! Only drives while Nunchuck button C is pressed
bool use_autonomus = false;
///value to tell if nunchuk c is pressed
bool nunchuk_c_pressed = false;
///last orientation value to try to follow if not autonomous driving is used
double last_orientation = 0;
///last linear value
double last_linear = 0;
///counter so it doesnt try to follow forever
int counter_try_to_follow = 0;
///counter so it actually drives backwards
int counter_back = 0;
///time it should try to follow
#define MAX_SEC_TRY_TO_FOLLOW 2

#define WII_BUTTON_NUNCHUK_C 1

/**
 * @brief build_servo builds servo-message with respect to linear and orientation
 * @param linear positiv if it should drive forward, near 0 if it should stop, negativ if drive backwards
 * @param orientation orientation if it should drive left or right
 * @return servo-message which should be published to /servo
 */
geometry_msgs::Vector3 build_servo(double linear, double orientation){
    geometry_msgs::Vector3 returnServo;
    //linear PID controller value

    if(linear>0.5){
        counter_back = 0;
        returnServo.x = 1550 + linear/2;
        if(returnServo.x > 1650){
            //that would be too fast
            returnServo.x = 1650;
        }
    }else if(linear < -0.5){
        counter_back ++;
        returnServo.x = 1270 + linear/2;
        if(returnServo.x < 1220){
            //that would be too fast
            returnServo.x = 1220;
        }
    }else{
        counter_back = 0;
        returnServo.x = 1500;
    }
    if(counter_back > 5 && counter_back < 10){
        returnServo.x = 1500;
    }

    returnServo.y = 1500+orientation*700/ANGLE_MAX*linear/fabs(linear); //switch orientation if driving backwards!
    ROS_INFO_STREAM("Servox: " << returnServo.x << " Servoy: " << returnServo.y);
    last_orientation = orientation;
    last_linear = linear;

    return returnServo;
}

/**
 * @brief build_goal builds move_baseage with respect to linear and orientation
 * @param linear positiv if it should drive forward, near 0 if it should stop, negativ if drive backwards
 * @param orientation orientation if it should drive left or right
 * @return move_base message which should be published to /move_base_simple/goal
 */
move_base_msgs::MoveBaseGoal build_goal(double linear, double orientation){
    double linear_goal = linear*0.0375; //40 entspricht 1.5m
    ROS_INFO_STREAM("building goal with lienar: " << linear_goal << " and orientation: " << orientation);
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.seq = num;

    goal.target_pose.pose.position.x = linear_goal;
    goal.target_pose.pose.position.y = linear_goal*sin(orientation);

    tf::Quaternion ori = tf::createQuaternionFromYaw(orientation);
    memcpy(&goal.target_pose.pose.orientation, &ori, sizeof(ori));
    num++;
    return goal;
}

/**
 * @brief get_linear_goal_y builds linear speed with respect to the y-coordinate of the center of the circle
 * @param circle circle which shall be followed
 * @return linear speed
 */
double get_linear_goal_y(Circle circle){
    //compare circle.r with desired_r
    //P controller

    //diff
    double diff_y = desired_y - circle.y;


    double linear_goal = KPY*diff_y;
    return linear_goal; // 0 = 40 cm //r=20 ~ 1.5m linear = 40
}


/**
 * @brief get_linear_goal builds linear speed with respect to the radius of the circle
 * @param circle circle which shall be followed
 * @return linear speed
 */
double get_linear_goal(Circle circle){
    //compare circle.r with desired_r
    //PID controller

    //diff
    double diff_r = DESIRED_R - circle.r;

    //integral part
    double int_diff = 0;
    for(int i = 0; i < NUMBER_INT; i++){
        int_diff += linear_int[i];
    }

    //diff part
    double diff_diff = diff_r - linear_int[linear_int_idx];

    //update linear_int
    linear_int_idx++;
    if(linear_int_idx == NUMBER_INT)
        linear_int_idx = 0;

    linear_int[linear_int_idx] = diff_r;

    double linear_goal = KP*diff_r + KI*int_diff + KD*diff_diff;
    return linear_goal; // 0 = 40 cm //r=20 ~ 1.5m linear = 40
}

/**
 * @brief get_orientation builds orientation with respect to the x-coordinate of the center of the circle
 * @param circle circle which shall be followed
 * @return orientation which shall be achieved
 */
double get_orientation(Circle circle){
    //quadratic
    double diff_x = desired_x - circle.x; //positiv -> orientation to the left

    double angle = ANGLE_MAX*(OA*pow(diff_x,2) + OB*diff_x)/(OA*pow(desired_x, 2) + OB*desired_x)*diff_x/fabs(diff_x); // max_angle*controller/normalize*prefix
    return angle;

}

/**
 * @brief wiiCallback callback for wii-controller, sets nunchuk_c_pressed
 * @param wiiState state of the wii-controller
 */
void wiiCallback(const wiimote::State::ConstPtr& wiiState){
    if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_C]==1){
        nunchuk_c_pressed = true;
    }else{
        nunchuk_c_pressed = false;
    }
}

void scanCallback(const sensor_msgs::LaserScanConstPtr &scan){
    stop = false;
    for(size_t i = 0; i<scan->ranges.size();i++){
        if(scan->ranges[i]<RANGE_MIN){
            stop = true;
        }
    }

}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  ros::init(argc, argv, "navigation_goals");
  ros::NodeHandle nh;

  /* create balltracker */
  FindBall findball(0);

  if(!findball.camera_found_){
      ROS_WARN("No camera! Exiting now!");
      return 0;
  }

  /* set x,y coord when car should stand still */
  desired_x = (double) findball.getWidth() * 0.5;
  desired_y = findball.getHeight() * DESIRED_Y;


  /* rate which shall be progressed */
  int hz_rate = 30;
  if(use_autonomus){
    hz_rate = 4;
  }
  ros::Rate rate(hz_rate);


  /* init PID controller */
  for(int i = 0; i < NUMBER_INT; i++){
      linear_int.push_back(0.0);
  }

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher servo_pub = nh.advertise<geometry_msgs::Vector3>("/servo_raw", 1);
  ros::Subscriber wii_sub = nh.subscribe("wiimote/state", 100, &wiiCallback);
  ros::Subscriber scan_sub = nh.subscribe("scan", 100, &scanCallback);

  if(use_autonomus){
      //wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }
  }


  while(ros::ok()){
      if(use_autonomus){
          //get ball
          Circle foundC = findball.consistentCircle(show);

          if(foundC.r != 0){
              //only valid if a circle was found, else circle will have radius 0

                  move_base_msgs::MoveBaseGoal goal = build_goal(get_linear_goal(foundC), get_orientation(foundC));

                  /* cancle all goals and send new */
                  ac.cancelAllGoals();
                  ROS_INFO("Sending goal");
                  ac.sendGoal(goal);

          }
      }else{
          //get ball
          std::vector<Circle> foundCA = findball.getAllCircles(show);
          geometry_msgs::Vector3 servo;
          bool pub = false;

          if(foundCA.size() != 0){
              //ball found! -> follow
              servo = build_servo(get_linear_goal(foundCA[0]), get_orientation(foundCA[0]));
              pub = true;
              counter_try_to_follow = 0;
          }else{
              //ball not found! -> check if it should try to follow it

              if(counter_try_to_follow < MAX_SEC_TRY_TO_FOLLOW*hz_rate){
                  //trying to find ball, turning slowly into the direction of last seen ball
                  double ori = 0;
                  if(last_orientation>0){
                      ori = ANGLE_MAX;
                  }else{
                      ori = -ANGLE_MAX;
                  }
                  if(last_linear>0){
                      servo = build_servo(11, last_orientation);
                      pub = true;
                  }else if(last_linear < 0){
                      servo = build_servo(-30, last_orientation);
                      pub = true;
                  }
                  counter_try_to_follow++;
              }
          }
          //publish everything
          if(pub){
              if(nunchuk_c_pressed){
                  ROS_INFO("Publishing Servo!");
                  if(!stop){
                    servo_pub.publish(servo);
                  }else{
                      servo.x = 1300;
                      stop = false;
                      servo_pub.publish(servo);
                  }
              }else{
                  ROS_INFO("Press nunchuk button C to drive automatically!");
              }
          }
      }

      rate.sleep();


      ros::spinOnce();
  }
  return 0;
}
