/*
 * dxl_control_node.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: ubuntu-ti
 */

#include "dxl_control.h"

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

double t = 0.0;
int goal_0 = 0;
int goal_1 = 0;

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
ros::Subscriber sub_joint_command_;

void DynamixelControl::CallBackOfJointCommand(const snake_msgs::snake_joint_command joint_command)
{
	  snake_msgs::snake_joint_command jc = joint_command;  // 名前を短く
	  if (joint_command.joint_index >= NUM_JOINT) {        // 対象が不適
	    ROS_WARN("joint_index %d is not exist.", joint_command.joint_index);
	    return;
	  }

	  if(joint_command.ping){DynamixelPing(); }

	  if(joint_command.change_mode_to_active){
		  DynamixelTorqueEnable();
	  }else if(joint_command.change_mode_to_free){
		  DynamixelTorqueDisable();
	  }

	  if(joint_command.set_position){DynamxielGoalPosition(joint_command.joint_index,
			  (joint_command.target_position*4095)/360+2048); }

}


int DynamixelControl::DynamixelOpenPort()
{

	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Open port
	if (portHandler->openPort())
	{
		ROS_INFO("Succeeded to open the port!\n");
	}
	else
	{
		ROS_INFO("Failed to open the port!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		ROS_INFO("Succeeded to change the baudrate!\n");
	}
	else
	{
		ROS_INFO("Failed to change the baudrate!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		return 0;
	}
	return 1;
}

int DynamixelControl::DynamixelTorqueEnable()
{
	int dxl_comm_result = COMM_TX_FAIL;   // Communication result
	uint8_t dxl_error = 0;                // Dynamixel error

   	// Enable Dynamixel#i Torque
    for(int i=0; i<NUM_OF_MOTOR; i++){

    	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_XH_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    	if (dxl_comm_result != COMM_SUCCESS)
    	{
    		packetHandler->printTxRxResult(dxl_comm_result);
    		return 0;
    	}
    	else if (dxl_error != 0)
    	{
    		packetHandler->printRxPacketError(dxl_error);
    		return 0;
    	}
    	else
    	{
    		ROS_INFO("DXL#%d has been successfully connected and TORQUE ON", i);
    	}
    }
    return 1;
}

int DynamixelControl::DynamixelTorqueDisable()
{
	int dxl_comm_result = COMM_TX_FAIL;     // Communication result
	uint8_t dxl_error = 0;                  // Dynamixel error

	for(int i=0; i<NUM_OF_MOTOR; i++){
		// Disable Dynamixel#1 Torque
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_XH_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    	if (dxl_comm_result != COMM_SUCCESS)
    	{
    		packetHandler->printTxRxResult(dxl_comm_result);
    	}
    	else if (dxl_error != 0)
    	{
    		packetHandler->printRxPacketError(dxl_error);
    	}else
    	{
    		ROS_INFO("DXL#%d has been successfully connected and TORQUE OFF", i);
    	}
    }
}

int DynamixelControl::DynamixelPing()
{
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl_model_number;                      // Dynamixel model number

	// Try to ping the Dynamixel
	// Get Dynamixel model number

	for(int i=0; i<NUM_OF_MOTOR; i++){
	  dxl_comm_result = packetHandler->ping(portHandler, i, &dxl_model_number, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS){
	    packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0){
	    packetHandler->printRxPacketError(dxl_error);
	  }

	  ROS_INFO("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", i, dxl_model_number);

	  // Close port
	  portHandler->closePort();

	}
	return 0;
}


/*  モータを目標位置まで動かす, SYNC でGoal Positionのデータを送る
  *  まとめて送るではなく，一つ一つで送る
 *  id: モータのID
 *  goalPos: ゴールポジション
 *
 * */
int DynamixelControl::DynamxielGoalPosition(int id, int goalPos)
{
	// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_XH_GOAL_POSITION, LEN_XH_GOAL_POSITION);
	// Initialize Groupsyncread instance for Present Position
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);

	int  dxl_comm_result      = COMM_TX_FAIL;         // Communication result
	bool dxl_addparam_result  = false;                // addParam result
	uint8_t param_goal_position[4];

	if(goalPos < 1024) goalPos=1024;
	if(goalPos > 3072) goalPos=3072;

	// Allocate goal position value into byte array
	param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goalPos));
	param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goalPos));
	param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goalPos));
	param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goalPos));

	// Add parameter storage for Dynamixel#id goal position
	dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_position);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", id);
		return 0;
	}
	// SyncWrite goal position
	dxl_comm_result = groupSyncWrite.txPacket();

	if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

	// Clear bulkWrite parameter storage
	groupSyncWrite.clearParam();
	return 1;
}

