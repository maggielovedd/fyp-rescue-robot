#include "Huanyu_robot.h"

/***
 @ Description	-> 
 @ Param		-> 6D 
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> Huanyu_start_object()
***/

Huanyu_start_object::Huanyu_start_object()
{
	memset(&Reciver_Str, 0, sizeof(Reciver_Str));
	memset(&Send_Str, 0, sizeof(Send_Str));
	x = y = th = vx = vy = vth = dt = 0.0;
	this->Gyroscope_Xdata_Offset = 0.0f; 
  	this->Gyroscope_Ydata_Offset = 0.0f; 
  	this->Gyroscope_Zdata_Offset = 0.0f;

  	this->Offest_Count = 0;

	this->Send_Str.Sensor_Str.Header = RECIVER_DATA_HEADER;
	this->Send_Str.Sensor_Str.End_flag = RECIVER_DATA_CHECK_SUM;

	/* Get Luncher file define value */
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("usart_port", this->usart_port, "/dev/huanyu_base"); 
   	nh_private.param<int>("baud_data", this->baud_data, 115200); 
   	nh_private.param<std::string>("robot_frame_id", this->robot_frame_id, "base_link");
	nh_private.param<std::string>("smoother_cmd_vel", this->smoother_cmd_vel, "/smoother_cmd_vel"); 
	nh_private.param<bool>("imu_velocity_z", this->imu_velocity_z, true); 

	nh_private.param<float>("filter_Vx_match", this->filter_Vx_match, 1.0f); 
	nh_private.param<float>("filter_Vth_match", this->filter_Vth_match, 1.0f); 

	/* Create a boot node for the underlying driver layer of the robot base_controller */
	this->cmd_vel_sub = n.subscribe(smoother_cmd_vel, 100, &Huanyu_start_object::cmd_velCallback, this);
	this->amcl_sub = n.subscribe("/amcl_pose", 100, &Huanyu_start_object::cmd_AmclvelCallback, this);

	this->odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	this->imu_pub  = n.advertise<sensor_msgs::Imu>("/mobile_base/sensors/imu_data", 20);
    this->imu_pub_raw  = n.advertise<sensor_msgs::Imu>("/mobile_base/sensors/imu_data_raw", 20);
    this->power_pub = n.advertise<std_msgs::Float32>("/robot/PowerValtage", 20);

	/**open seril device**/
	try{
         Robot_Serial.setPort(this->usart_port);
         Robot_Serial.setBaudrate(this->baud_data);
         serial::Timeout to = serial::Timeout::simpleTimeout(2000);
         Robot_Serial.setTimeout(to);
         Robot_Serial.open();
    }
	catch (serial::IOException& e){
		 ROS_ERROR_STREAM("[ZHOUXUEWEI] Unable to open port ");
	}
	if(Robot_Serial.isOpen()){
	 	ROS_INFO_STREAM("[ZHOUXUEWEI] Serial Port opened");
	}else{
	}
}

Huanyu_start_object::~Huanyu_start_object()
{
	Robot_Serial.close();
}
/***
 @ Description	-> cmd_vel Callback function
 @ Param		-> const geometry_msgs::Twist &twist_aux 
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> Huanyu_start_object::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
***/
void Huanyu_start_object::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	/** process callback function msgs**/
	
	Send_Str.Sensor_Str.X_speed = twist_aux.linear.x;
	Send_Str.Sensor_Str.Z_speed = twist_aux.angular.z;

	Robot_Serial.write(Send_Str.buffer, sizeof(Send_Str.buffer));
}
/***
 @ Description	-> amcl pose Callback function
 @ Param		-> const geometry_msgs::Twist &twist_aux 
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     ->  Huanyu_start_object::cmd_AmclvelCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose)
***/
void Huanyu_start_object::cmd_AmclvelCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &Pose)
{
	geometry_msgs::Pose Amclpose;
	float yaw_angle = 0.0f;

	Amclpose.position.x = Pose->pose.pose.position.x;
	Amclpose.position.y = Pose->pose.pose.position.y;

	Amclpose.orientation = Pose->pose.pose.orientation;
	yaw_angle = tf::getYaw(Amclpose.orientation);
}
/***
 @ Description	-> Publisher Odom
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> void Huanyu_start_object::PublisherOdom()
***/
void Huanyu_start_object::PublisherOdom()
{
	
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	std_msgs::Float32 power_msgs;
	power_msgs.data = this->Power_valtage;
	power_pub.publish(power_msgs);

	//first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = ros::Time::now();;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = this->robot_frame_id;

    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_quat;
    // //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

	
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = this->robot_frame_id;
    odom.twist.twist.linear.x =  this->vx;
    odom.twist.twist.linear.y =  this->vy;
    odom.twist.twist.angular.z = this->vth;		

    if(this->vx == 0)
    {
    	memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
    	memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    }
    else
    {
    	memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
    	memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
    }				

    //publish the message
    odom_pub.publish(odom);
}

/***
 @ Description	-> send and get stm32 board data
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> bool Huanyu_start_object::ReadFormUart()
 @ return 		-> status
***/
bool Huanyu_start_object::ReadFormUart()
{	
	unsigned char CheckSumBuffer[1];

	Robot_Serial.read(Reciver_Str.buffer,sizeof(Reciver_Str.buffer));

	if (Reciver_Str.Sensor_Str.Header == RECIVER_DATA_HEADER)
	{
		if (Reciver_Str.Sensor_Str.End_flag == RECIVER_DATA_CHECK_SUM)
		{
			/* Get robot speed value */
			this->vx =  Reciver_Str.Sensor_Str.X_speed * filter_Vx_match;

			(!this->imu_velocity_z)?\
			(this->vth = -Reciver_Str.Sensor_Str.Z_speed * filter_Vth_match):\
			(this->vth = Mpu6050.angular_velocity.z);

			this->Power_valtage = Reciver_Str.Sensor_Str.Source_Voltage;

			Mpu6050.linear_acceleration.x = Reciver_Str.Sensor_Str.Link_Accelerometer.X_data / ACCELEROMETER;
			Mpu6050.linear_acceleration.y = Reciver_Str.Sensor_Str.Link_Accelerometer.Y_data / ACCELEROMETER;
			Mpu6050.linear_acceleration.z = Reciver_Str.Sensor_Str.Link_Accelerometer.Z_data / ACCELEROMETER;

			Mpu6050.angular_velocity.x = Reciver_Str.Sensor_Str.Link_Gyroscope.X_data * GYROSCOPE_RADIAN;
			Mpu6050.angular_velocity.y = Reciver_Str.Sensor_Str.Link_Gyroscope.Y_data * GYROSCOPE_RADIAN;
			Mpu6050.angular_velocity.z = Reciver_Str.Sensor_Str.Link_Gyroscope.Z_data * GYROSCOPE_RADIAN;

			return true;
		}
	} 
	Robot_Serial.read(CheckSumBuffer,sizeof(CheckSumBuffer));
	//ROS_INFO("[ZHOUXUEWEI] Get base controller data error!");
	return false;
}

/***
 @ Description	-> publish /imu_data_raw
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-25
 @ Function     -> void Huanyu_start_object::publisherImuSensorRaw()
 @ return 		-> null
***/
void Huanyu_start_object::publisherImuSensorRaw()
{
	sensor_msgs::Imu ImuSensorRaw;

	ImuSensorRaw.header.stamp = ros::Time::now(); 
	ImuSensorRaw.header.frame_id = "gyro_link"; 

	ImuSensorRaw.orientation.x = 0; 
	ImuSensorRaw.orientation.y = 0; 
	ImuSensorRaw.orientation.z = 0; 
	ImuSensorRaw.orientation.w = 0; 

	ImuSensorRaw.angular_velocity.x = Mpu6050.angular_velocity.x; 
	ImuSensorRaw.angular_velocity.y = Mpu6050.angular_velocity.y; 
	ImuSensorRaw.angular_velocity.z = Mpu6050.angular_velocity.z;

	ImuSensorRaw.linear_acceleration.x = 0; 
	ImuSensorRaw.linear_acceleration.y = 0; 
	ImuSensorRaw.linear_acceleration.z = 0;  	

	imu_pub_raw.publish(ImuSensorRaw); 
}
/***
 @ Description	-> publish /imu_data
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-25
 @ Function     -> void Huanyu_start_object::publisherImuSensor()
 @ return 		-> null
***/
void Huanyu_start_object::publisherImuSensor()
{
	sensor_msgs::Imu ImuSensor;

	ImuSensor.header.stamp = ros::Time::now(); 
	ImuSensor.header.frame_id = "gyro_link"; 

	ImuSensor.orientation.x = 0.0; 
	ImuSensor.orientation.y = 0.0; 
	ImuSensor.orientation.z = Mpu6050.orientation.z;
	ImuSensor.orientation.w = Mpu6050.orientation.w;

	ImuSensor.orientation_covariance[0] = 1e6;
	ImuSensor.orientation_covariance[4] = 1e6;
	ImuSensor.orientation_covariance[8] = 1e-6;

	ImuSensor.angular_velocity.x = 0.0;		
	ImuSensor.angular_velocity.y = 0.0;		
	ImuSensor.angular_velocity.z = Mpu6050.angular_velocity.z;

	ImuSensor.angular_velocity_covariance[0] = 1e6;
	ImuSensor.angular_velocity_covariance[4] = 1e6;
	ImuSensor.angular_velocity_covariance[8] = 1e-6;

	ImuSensor.linear_acceleration.x = 0; 
	ImuSensor.linear_acceleration.y = 0; 
	ImuSensor.linear_acceleration.z = 0;  

	imu_pub.publish(ImuSensor); 
}

/***
 @ Description	-> loop 
 @ Param		-> null
 @ Author		-> XueweiZhou
 @ Date			-> 2019-03-10
 @ Function     -> bool Huanyu_start_object::ReadAndWriteLoopProcess()
 @ return 		-> status
***/
bool Huanyu_start_object::ReadAndWriteLoopProcess()
{
	this->last_time = ros::Time::now();

	while(ros::ok())
	{
		this->current_time = ros::Time::now();
		this->dt = (current_time - last_time).toSec();

		if (true == ReadFormUart()) 	/* Get npu data include robot move speed and action status information*/
		{
			/* Calculation tf and odom */
			double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
			double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
			double delta_th = vth * dt;
			x += delta_x;
			y += delta_y;
			th += delta_th;		

			if (Offest_Count < OFFSET_COUNT)
			{
				Offest_Count++;
				accelerometerOffset(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z);
			}
			else
			{
				Offest_Count = OFFSET_COUNT;
				Mpu6050.angular_velocity.x = Mpu6050.angular_velocity.x - this->Gyroscope_Xdata_Offset;
				Mpu6050.angular_velocity.y = Mpu6050.angular_velocity.y - this->Gyroscope_Ydata_Offset;
				Mpu6050.angular_velocity.z = Mpu6050.angular_velocity.z - this->Gyroscope_Zdata_Offset;

				MahonyAHRSupdateIMU(0.0, 0.0, Mpu6050.angular_velocity.z,\
								0.0, 0.0, Mpu6050.linear_acceleration.z);

				PublisherOdom();				/* Publisher odom topic */
				publisherImuSensor();
				publisherImuSensorRaw();
			}
		}
		this->last_time = current_time;
		ros::spinOnce();
	}
}

float Huanyu_start_object::invSqrt(float number)
{
	volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );

	return y;
}


void Huanyu_start_object::accelerometerOffset(float gx, float gy, float gz)
{
	this->Gyroscope_Xdata_Offset += gx; 
  	this->Gyroscope_Ydata_Offset += gy; 
  	this->Gyroscope_Zdata_Offset += gz;

  	if (Offest_Count == OFFSET_COUNT)
  	{
  		this->Gyroscope_Xdata_Offset = this->Gyroscope_Xdata_Offset / OFFSET_COUNT;
  		this->Gyroscope_Ydata_Offset = this->Gyroscope_Ydata_Offset / OFFSET_COUNT;
  		this->Gyroscope_Zdata_Offset = this->Gyroscope_Zdata_Offset / OFFSET_COUNT;
  	}
}



volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

void Huanyu_start_object::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// 把四元数换算成方向余弦中的第三行的三个元素
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;				// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;				// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	Mpu6050.orientation.w = q0;
	Mpu6050.orientation.x = q1;
	Mpu6050.orientation.y = q2;
	Mpu6050.orientation.z = q3;
}

int main(int argc, char** argv)
{
	/* Voltage thread fb*/

	ros::init(argc, argv, "base_controller");
	ROS_INFO("[ZHOUXUEWEI] base controller node start! ");

	Huanyu_start_object Robot_Control; 
	Robot_Control.ReadAndWriteLoopProcess();
	
	return 0;
}


