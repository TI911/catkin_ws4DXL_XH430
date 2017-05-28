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
    		ROS_INFO("DXL#%d has been successfully connected \n", i);
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
	  dxl_comm_result = packetHandler->ping(portHandler, DXL1_ID, &dxl_model_number, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0)
	  {
	    packetHandler->printRxPacketError(dxl_error);
	  }

	  ROS_INFO("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", DXL1_ID, dxl_model_number);

	  // Close port
	  portHandler->closePort();

	  return 0;
}

int DynamixelControl::DynamixelBulkReadWrite()
{
	// Initialize GroupBulkWrite instance
	dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

	// Initialize GroupBulkRead instance
	dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

	int  index = 0;
	int  dxl_comm_result      = COMM_TX_FAIL;         // Communication result
	bool dxl_addparam_result  = false;                // addParam result
	bool dxl_getdata_result   = false;                // GetParam result
	int  dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};      // Goal position

	uint8_t dxl_error        = 0;             // Dynamixel error
	uint8_t dxl_led_value[2] = {0, 1};        // Dynamixel LED value for write
	uint8_t param_goal_position[4];
	int32_t dxl1_present_position = 0;        // Present position
	uint8_t dxl2_led_value_read; 		  	  // Dynamixel LED value for read

	// Enable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
	else
	{
		ROS_INFO("DXL#%d has been successfully connected \n", DXL1_ID);
	}

	// Enable Dynamixel#2 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
	else
	{
		ROS_INFO("DXL#%d has been successfully connected \n", DXL2_ID);
	}

	// Add parameter storage for Dynamixel#1 present position
	dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL1_ID);
		return 0;
	}

	// Add parameter storage for Dynamixel#2 LED value
	dxl_addparam_result = groupBulkRead.addParam(DXL2_ID, ADDR_XH_LED_RED, LEN_XH_LED_RED);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL2_ID);
		return 0;
	}

	while(1)
	{
		ROS_INFO("Press any key to continue! (or press ESC to quit!)\n");
		if (getch() == ESC_ASCII_VALUE)
			break;

	    // Allocate goal position value into byte array
	    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
	    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
	    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
	    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

	    // Add parameter storage for Dynamixel#1 goal position
	    dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_XH_GOAL_POSITION, LEN_XH_GOAL_POSITION, param_goal_position);
	    if (dxl_addparam_result != true)
	    {
	      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL1_ID);
	      return 0;
	    }

	    // Add parameter storage for Dynamixel#2 LED value
	    dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_XH_LED_RED, LEN_XH_LED_RED, &dxl_led_value[index]);
	    if (dxl_addparam_result != true)
	    {
	      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL2_ID);
	      return 0;
	    }

	    // Bulkwrite goal position and LED value
	    dxl_comm_result = groupBulkWrite.txPacket();

	    if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

	    // Clear bulkwrite parameter storage
	    groupBulkWrite.clearParam();

	    do
	    {
	      // Bulkread present position and LED status
	      dxl_comm_result = groupBulkRead.txRxPacket();
	      if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

	      // Check if groupbulkread data of Dynamixel#1 is available
	      dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID);
	        return 0;
	      }

	      // Check if groupbulkread data of Dynamixel#2 is available
	      dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_XH_LED_RED, LEN_XH_LED_RED);
	      if (dxl_getdata_result != true)
	      {
	        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL2_ID);
	        return 0;
	      }

	      // Get present position value
	      dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);

	      // Get LED value
	      dxl2_led_value_read = groupBulkRead.getData(DXL2_ID, ADDR_XH_LED_RED, LEN_XH_LED_RED);

	      ROS_INFO("[ID:%03d] Present Position : %d \t [ID:%03d] LED Value: %d\n", DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_led_value_read);

	    }while(abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD);

	    // Change goal position
	    if (index == 0)
	    {
	    	index = 1;
	    }
	    else
	    {
	    	index = 0;
	    }
	}

	  // Disable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	// Disable Dynamixel#2 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}

	// Close port
	portHandler->closePort();

	return 0;

}

int DynamixelControl::DynamixelReadWrite()
{
	  int index = 0;
	  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

	  uint8_t dxl_error = 0;                          // Dynamixel error
	  int32_t dxl_present_position = 0;               // Present position

	  // Enable Dynamixel Torque
	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0)
	  {
	    packetHandler->printRxPacketError(dxl_error);
	  }
	  else
	  {
	    printf("Dynamixel has been successfully connected \n");
	  }

	  while(1)
	  {
	    printf("Press any key to continue! (or press ESC to quit!)\n");
	    if (getch() == ESC_ASCII_VALUE)
	      break;

	    // Write goal position
	    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XH_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }

	    do
	    {
	      // Read present position
	      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XH_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
	      if (dxl_comm_result != COMM_SUCCESS)
	      {
	        packetHandler->printTxRxResult(dxl_comm_result);
	      }
	      else if (dxl_error != 0)
	      {
	        packetHandler->printRxPacketError(dxl_error);
	      }

	      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);

	    }while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

	    // Change goal position
	    if (index == 0)
	    {
	      index = 1;
	    }
	    else
	    {
	      index = 0;
	    }
	  }

	  // Disable Dynamixel Torque
	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
	    packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0)
	  {
	    packetHandler->printRxPacketError(dxl_error);
	  }

	  // Close port
	  portHandler->closePort();

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

