/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_keyboard_input.cpp
 *      Project   : moveit2_tutorials
 *      Created   : 05/31/2021
 *      Author    : Adam Pettinger
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
// #define KEYCODE_RIGHT 0x43
// #define KEYCODE_LEFT 0x44
// #define KEYCODE_UP 0x41
// #define KEYCODE_DOWN 0x42
// #define KEYCODE_PERIOD 0x2E
// #define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_0 0x30
#define KEYCODE_MINUS 0x2d
#define KEYCODE_EQUAL 0x3d

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7a
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63

#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_O 0x6f
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_M 0x6d
#define KEYCODE_COMMA 0x2c
#define KEYCODE_END 0x2e

#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_Y 0x79

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC1 = "/servo_node1/delta_twist_cmds";
const std::string JOINT_TOPIC1 = "/servo_node1/delta_joint_cmds";
const std::string TWIST_TOPIC2 = "/servo_node2/delta_twist_cmds";
const std::string JOINT_TOPIC2 = "/servo_node2/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
// const std::string EEF_FRAME_ID = "robot1_wrist3_link";
std::string BASE_FRAME_ID1 = "robot1_base_link";
std::string BASE_FRAME_ID2 = "robot2_base_link";

const int N = 4; //number of repeats for each key press
const double interval = 0.008;

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }

  void flush()
  {
      tcflush(kfd, TCIFLUSH);
  }
  
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  void spin();

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_1;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_1;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_2;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_2;

  // std::string frame_to_publish_;
  double joint_vel_cmd_;
};

// KeyboardServo::KeyboardServo() : frame_to_publish_(BASE_FRAME_ID), joint_vel_cmd_(1.0)
KeyboardServo::KeyboardServo() : joint_vel_cmd_(1.0)

{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  twist_pub_1 = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC1, ROS_QUEUE_SIZE);
  joint_pub_1 = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC1, ROS_QUEUE_SIZE);
  twist_pub_2 = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC2, ROS_QUEUE_SIZE);
  joint_pub_2 = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC2, ROS_QUEUE_SIZE);
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist1 = false;
  bool publish_joint1 = false;
  bool publish_twist2 = false;
  bool publish_joint2 = false;

  // rclcpp::Rate rate(std::chrono::milliseconds(static_cast<int>(interval*1000)));

  std::thread{ std::bind(&KeyboardServo::spin, this) }.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
  puts("Use 1|2|3|4|5|6 keys to joint jog. 'R' to reverse the direction of jogging.");
  puts("'Q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
      input.flush();
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg1 = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg1 = std::make_unique<control_msgs::msg::JointJog>();
    auto twist_msg2 = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg2 = std::make_unique<control_msgs::msg::JointJog>();

    // Use read key-press
    switch (c)
    {
      case KEYCODE_A:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg1->twist.linear.z = 1.0;
        publish_twist1 = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg1->twist.linear.z = -1.0;
        publish_twist1 = true;
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg1->twist.linear.y = -1.0;
        publish_twist1 = true;
        break;
      case KEYCODE_S:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg1->twist.linear.y = 1.0;
        publish_twist1 = true;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg1->twist.linear.x = -1.0;
        publish_twist1 = true;
        break;
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg1->twist.linear.x = 1.0;
        publish_twist1 = true;
        break;
      case KEYCODE_Z:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg1->twist.angular.z = joint_vel_cmd_;
        publish_twist1 = true;
        break;
      case KEYCODE_X:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg1->twist.angular.y = -joint_vel_cmd_;
        publish_twist1 = true;
        break;
      case KEYCODE_C:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg1->twist.angular.x = -joint_vel_cmd_;
        publish_twist1 = true;
        break;
      case KEYCODE_1:
        RCLCPP_DEBUG(nh_->get_logger(), "1");
        joint_msg1->joint_names.push_back("robot1_j1");
        joint_msg1->velocities.push_back(joint_vel_cmd_);
        publish_joint1 = true;
        break;
      case KEYCODE_2:
        RCLCPP_DEBUG(nh_->get_logger(), "2");
        joint_msg1->joint_names.push_back("robot1_j2");
        joint_msg1->velocities.push_back(joint_vel_cmd_);
        publish_joint1 = true;
        break;
      case KEYCODE_3:
        RCLCPP_DEBUG(nh_->get_logger(), "3");
        joint_msg1->joint_names.push_back("robot1_j3");
        joint_msg1->velocities.push_back(joint_vel_cmd_);
        publish_joint1 = true;
        break;
      case KEYCODE_4:
        RCLCPP_DEBUG(nh_->get_logger(), "4");
        joint_msg1->joint_names.push_back("robot1_j4");
        joint_msg1->velocities.push_back(joint_vel_cmd_);
        publish_joint1 = true;
        break;
      case KEYCODE_5:
        RCLCPP_DEBUG(nh_->get_logger(), "5");
        joint_msg1->joint_names.push_back("robot1_j5");
        joint_msg1->velocities.push_back(joint_vel_cmd_);
        publish_joint1 = true;
        break;
      case KEYCODE_6:
        RCLCPP_DEBUG(nh_->get_logger(), "6");
        joint_msg1->joint_names.push_back("robot1_j6");
        joint_msg1->velocities.push_back(joint_vel_cmd_);
        publish_joint1 = true;
        break;



      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "R");
        joint_vel_cmd_ *= -1;
        break;
      case KEYCODE_T:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        BASE_FRAME_ID1 = "robot1_base_link";
        BASE_FRAME_ID2 = "robot2_base_link";
        break;
      case KEYCODE_Y:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        BASE_FRAME_ID1 = "robot1_wrist3_link";
        BASE_FRAME_ID2 = "robot2_wrist3_link";
        break;



      case KEYCODE_J:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg2->twist.linear.z = -1.0;
        publish_twist2 = true;
        break;
      case KEYCODE_L:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg2->twist.linear.z = 1.0;
        publish_twist2 = true;
        break;
      case KEYCODE_I:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg2->twist.linear.x = -1.0;
        publish_twist2 = true;
        break;
      case KEYCODE_K:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg2->twist.linear.x = 1.0;
        publish_twist2 = true;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg2->twist.linear.y = -1.0;
        publish_twist2 = true;
        break;
      case KEYCODE_O:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg2->twist.linear.y = 1.0;
        publish_twist2 = true;
        break;
      case KEYCODE_M:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg2->twist.angular.z = -joint_vel_cmd_;
        publish_twist2 = true;
        break;
      case KEYCODE_COMMA:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg2->twist.angular.x = -joint_vel_cmd_;
        publish_twist2 = true;
        break;
      case KEYCODE_END:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg2->twist.angular.y = -joint_vel_cmd_;
        publish_twist2 = true;
        break;
      case KEYCODE_7:
        RCLCPP_DEBUG(nh_->get_logger(), "1");
        joint_msg2->joint_names.push_back("robot2_j1");
        joint_msg2->velocities.push_back(joint_vel_cmd_);
        publish_joint2 = true;
        break;
      case KEYCODE_8:
        RCLCPP_DEBUG(nh_->get_logger(), "2");
        joint_msg2->joint_names.push_back("robot2_j2");
        joint_msg2->velocities.push_back(joint_vel_cmd_);
        publish_joint2 = true;
        break;
      case KEYCODE_9:
        RCLCPP_DEBUG(nh_->get_logger(), "3");
        joint_msg2->joint_names.push_back("robot2_j3");
        joint_msg2->velocities.push_back(joint_vel_cmd_);
        publish_joint2 = true;
        break;
      case KEYCODE_0:
        RCLCPP_DEBUG(nh_->get_logger(), "4");
        joint_msg2->joint_names.push_back("robot2_j4");
        joint_msg2->velocities.push_back(joint_vel_cmd_);
        publish_joint2 = true;
        break;
      case KEYCODE_MINUS:
        RCLCPP_DEBUG(nh_->get_logger(), "5");
        joint_msg2->joint_names.push_back("robot2_j5");
        joint_msg2->velocities.push_back(joint_vel_cmd_);
        publish_joint2 = true;
        break;
      case KEYCODE_EQUAL:
        RCLCPP_DEBUG(nh_->get_logger(), "6");
        joint_msg2->joint_names.push_back("robot2_j6");
        joint_msg2->velocities.push_back(joint_vel_cmd_);
        publish_joint2 = true;
        break;   
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist1)
    {


      for (int i =0;i<N;i++){
        twist_msg1->header.stamp = nh_->now();
        twist_msg1->header.frame_id = BASE_FRAME_ID1;
        auto msg_copy = *twist_msg1;
        twist_pub_1->publish(msg_copy);
        usleep(interval * 1000000);
        // RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"time is %d, %f",i,twist_msg1->header.stamp.sec + twist_msg1->header.stamp.nanosec*1e-9);
        // rate.sleep();
      }
      publish_twist1 = false;
    }
    else if (publish_joint1)
    {

      for (int i =0;i<N;i++){
        joint_msg1->header.stamp = nh_->now();
        joint_msg1->header.frame_id = BASE_FRAME_ID1;
        auto msg_copy = *joint_msg1;
        joint_pub_1->publish(msg_copy);
        usleep(interval * 1000000);
        // RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"time is %d, %f",i,joint_msg1->header.stamp.sec + joint_msg1->header.stamp.nanosec*1e-9);
        // rate.sleep();
      }
      publish_joint1 = false;
    }
    else if (publish_twist2)
    {

      for (int i =0;i<N;i++){
        twist_msg2->header.stamp = nh_->now();
        twist_msg2->header.frame_id = BASE_FRAME_ID2;
        auto msg_copy = *twist_msg2;
        twist_pub_2->publish(msg_copy);
        usleep(interval * 1000000);
        // RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"time is %d, %f",i,twist_msg2->header.stamp.sec + twist_msg2->header.stamp.nanosec*1e-9);
        // rate.sleep();
      }
      publish_twist2 = false;
    }
    else if (publish_joint2)
    {
      joint_msg2->header.stamp = nh_->now();
      joint_msg2->header.frame_id = BASE_FRAME_ID2;
      for (int i =0;i<N;i++){
        auto msg_copy = *joint_msg2;
        joint_pub_2->publish(msg_copy);
        usleep(interval * 1000000);
        // RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"time is %d, %f",i,joint_msg2->header.stamp.sec + joint_msg2->header.stamp.nanosec*1e-9);
        // rate.sleep();
      }
      publish_joint2 = false;
    }
  }

  return 0;
}