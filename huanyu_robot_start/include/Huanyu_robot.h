
#ifndef __HUANYU_ROBOT_H_
#define __HUANYU_ROBOT_H_
// sudo apt-get install ros-melodic-serial
// rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}'
// sudo usermod -aG　dialout wsh

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>       
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace std;

#define RECIVER_DATA_HEADER 		0XFEFEFEFE
#define RECIVER_DATA_CHECK_SUM 		0XEE
#define PROTOBUF_SIZE				33

#define PI 					3.1415926f	
#define GYROSCOPE_RADIAN	0.001064f	// gyro_x/(16.40*57.30)=gyro_x*0.001064 单位为弧度每秒
#define GYROSCOPE_DEGREE 	16.40f		// Accelerometer_Xdata/16.40=61度每秒
#define ACCELEROMETER 		16384.0f  	// 1000/2048=0.49g。g为加速度的单位，重力加速度定义为1g, 等于9.8米每平方秒。

#define sampleFreq	20.0f				// sample frequency in Hz
#define twoKpDef	1.0f				// (2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	0.0f				// (2.0f * 0.0f)	// 2 * integral gain

#define OFFSET_COUNT 	40

const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
 
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};

#pragma pack(1)
typedef struct __Mpu6050_Str_
{
	short X_data;
	short Y_data;
	short Z_data;
}Mpu6050_Str;

typedef union _Upload_Data_   
{
	unsigned char buffer[PROTOBUF_SIZE];
	
	struct _Sensor_Str_
	{
		unsigned int Header;
		float X_speed;
		float Y_speed;
		float Z_speed;
		
		float Source_Voltage;
		
		Mpu6050_Str Link_Accelerometer;
		Mpu6050_Str Link_Gyroscope;
		
		unsigned char End_flag;
	}Sensor_Str;
}Upload_Data;
#pragma pack(4)


class Huanyu_start_object
{
	public:
		Huanyu_start_object();
		~Huanyu_start_object();

		/* /cmd_val topic call function */
		void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
		void cmd_AmclvelCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &Pose);


		/* Read/Write data from ttyUSB */
		bool ReadFormUart();
		bool WriteToUart(unsigned char*){}
		bool ReadAndWriteLoopProcess();

		/* This node Publisher topic and tf */
		void PublisherOdom();
		void publisherImuSensor();
		void publisherImuSensorRaw();

		float invSqrt(float number);
		/*输入参数为加速度xyz原始数据，磁力计xyz原始数据，陀螺仪数据必须转换为弧度制*/
		void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
		void accelerometerOffset(float gx, float gy, float gz);


		serial::Serial Robot_Serial; //声明串口对象 

	private:
		int baud_data;
		string usart_port, robot_frame_id, smoother_cmd_vel;
		float filter_Vx_match,filter_Vth_match;
		bool imu_velocity_z;
 
		/* Ros node define*/
		ros::NodeHandle n;
		ros::Time current_time, last_time;

		ros::Subscriber cmd_vel_sub, amcl_sub;
		ros::Publisher odom_pub, imu_pub, imu_pub_raw, power_pub;
		tf::TransformBroadcaster odom_broadcaster;

		Upload_Data Reciver_Str, Send_Str;         

		/* Odom and tf value*/
		double x, y, th, vx, vy, vth, dt;

		sensor_msgs::Imu Mpu6050;
		float Gyroscope_Xdata_Offset;
		float Gyroscope_Ydata_Offset;
		float Gyroscope_Zdata_Offset;
		float Power_valtage;

		unsigned short Offest_Count;
};


#endif


