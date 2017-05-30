/*
 * dxl_control.h
 *
 *  Created on: May 26, 2017
 *      Author: ubuntu-ti
 */

#ifndef DXL_CONTROL_SRC_DXL_CONTROL_H_
#define DXL_CONTROL_SRC_DXL_CONTROL_H_

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif



#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "dynamixel_sdk.h"

#include "std_msgs/String.h"
#include "snake_msgs/snake_joint_command.h"
//#include "joy_handler_hori/JoySelectedData.h"
using namespace dynamixel;

#define QUEUE_SIZE_JOINT    100
#define NUM_OF_MOTOR 20
#define NUM_JOINT 20


// Control table address
//#define ADDR_PRO_TORQUE_ENABLE          562        // DXL_PRO Control table address is different in Dynamixel model
//#define ADDR_PRO_LED_RED                563
//#define ADDR_PRO_GOAL_POSITION          596

#define ADDR_XH_TORQUE_ENABLE          64          // DXL_XH  Control table address is different in Dynamixel model
#define ADDR_XH_LED_RED                65
#define ADDR_XH_GOAL_POSITION          116
#define ADDR_XH_PRESENT_POSITION       132

// Data Byte Length
#define LEN_XH_LED_RED                 1
#define LEN_XH_GOAL_POSITION           4
#define LEN_XH_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0         // See which protocol version is used in the Dynamixel

// Default setting

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      1024  //-150000     // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3072  //150000      // and this value (note that the Dynamixel would not move when the position value is
															//out of movable range. Check e-manual about the range of the Dynamixel you use.)

#define DXL_MOVING_STATUS_THRESHOLD     123   //20                  // Dynamixel moving status threshold
#define ESC_ASCII_VALUE                 0x1b


class DynamixelControl{
public:
	static void Initialize(){

		ros::NodeHandle node;

	    //--- Subscriver ---//
	    // Command
		static ros::Subscriber sub_joint_command_ = node.subscribe("joint_command", QUEUE_SIZE_JOINT, DynamixelControl::CallBackOfJointCommand);

		DynamixelOpenPort();
	}

	static int DynamixelOpenPort();
	static int DynamixelTorqueEnable();
	static int DynamixelTorqueDisable();

	static int DynamixelPing();
	static int DynamixelBulkReadWrite();
	static int DynamixelReadWrite();
	static int DynamxielGoalPosition(int id, int goalPos);

private:
	// CallBack関数
	static void CallBackOfJointCommand(const snake_msgs::snake_joint_command joint_command);

	//static ros::Subscriber sub_joint_command_;
};

#endif /* DXL_CONTROL_SRC_DXL_CONTROL_H_ */
