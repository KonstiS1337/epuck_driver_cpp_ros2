#pragma once

#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv/cv.h>
extern "C" {
	#include <stdio.h>
	#include <stdint.h> 
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <linux/i2c-dev.h> /* for I2C_SLAVE */
	//#include <linux/i2c.h>
	#include <sys/ioctl.h>
	#include <stdlib.h>
	#include <unistd.h>
}

#define DEBUG_CONNECTION_INIT 1
#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_TIMING 0
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_COMMUNICATION_ERROR 1
#define DEBUG_ODOMETRY 0
#define DEBUG_IMU 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_LED_RECEIVED 0

#define I2C_CHANNEL "/dev/i2c-12"
#define LEGACY_I2C_CHANNEL "/dev/i2c-4"

#define READ_TIMEOUT_SEC 10    // 10 seconds, keep it high to avoid desynchronize when there are communication delays due to Bluetooth.
#define READ_TIMEOUT_USEC 0
#define MAX_CONSECUTIVE_TIMEOUT 3

#define SENSORS_NUM 10
#define IMU 0
#define MOTOR_SPEED 1
#define FLOOR 2
#define PROXIMITY 3
#define MOTOR_POSITION 4
#define MICROPHONE 5
#define CAMERA 6
#define TV_REMOTE 7
#define SELECTOR 8
#define TIME_OF_FLIGHT 9

#define ACTUATORS_NUM 4
#define MOTORS 0
#define LEDS 1
#define MOTORS_POS 2
#define SPEAKER 3

#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_SEPARATION 5.3    // Separation between wheels (cm).
#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

#define LEDS_NUM 4 		// Number of LEDs on the robot.
#define RGB_LEDS_NUM 4	// Number of RGB LEDs on the robot.

#define ACTUATORS_SIZE (19+1) // Data + checksum.
#define SENSORS_SIZE (46+1) // Data + checksum.
#define ROBOT_ADDR 0x1F

#define NUM_SAMPLES_CALIBRATION 20
#define AK8963_ADDRESS 0x0C		// Address of magnetometer
#define AK8963_XOUT_L 0x03		// data

#define MPU9250_ADDRESS_AD1_0 0x68  // Device address when AD1 = 0
#define MPU9250_ADDRESS_AD1_1 0x69

#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define TEMP_OUT_H         0x41
#define GYRO_XOUT_H        0x43

#define GRAVITY_MPU9250 16384 // 1 g for 16 bits in 2g scale mode
#define DEG2RAD(deg) (deg / 180 * M_PI)
#define STANDARD_GRAVITY 9.80665f
#define ACC_RAW2G (2.0/32768.0f)   //2G scale for int16 raw value
#define GYRO_RAW2DPS (250.0/32768.0f)   //250DPS (degrees per second) scale for int16 raw value

class PiPuckRos2 : public rclcpp::Node {
	public:
		PiPuckRos2();
		~PiPuckRos2();
		bool initialized() {return initialized_;};
	private:
		int fh;
		char zero_to_epuck_buff[ACTUATORS_SIZE]; 
		char epuck_to_zero_buff[SENSORS_SIZE];
		bool enabledSensors[SENSORS_NUM];
		bool changedActuators[ACTUATORS_NUM];
		bool initialized_ = false;
		int speedLeft = 0, speedRight = 0;
		unsigned char ledState[LEDS_NUM] = {0, 0, 0, 0};
		unsigned char rgbLedState[RGB_LEDS_NUM][3];
		int stepsLeft = 0, stepsRight = 0;
		std::string epuckname;
		struct timeval currentTime2, lastTime2;
		struct timeval currentTime3, lastTime3;
		int consecutiveReadTimeout = 0;

		int proxData[8];
		int motorPositionData[2];
		int micData[4];
		uint8_t selectorData;
		uint8_t tvRemoteData;

		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr proxPublisher[8];
		sensor_msgs::msg::Range proxMsg[8];
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPublisher;
		sensor_msgs::msg::LaserScan laserMsg;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
		nav_msgs::msg::Odometry odomMsg;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr microphonePublisher;
		visualization_msgs::msg::Marker microphoneMsg;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
		sensor_msgs::msg::Imu imuMsg;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr motorSpeedPublisher;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr floorPublisher;

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber;
		rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr cmdLedSubscriber;

		double leftStepsDiff = 0, rightStepsDiff = 0;
		double leftStepsPrev = 0, rightStepsPrev = 0;
		signed long int leftStepsRawPrev = 0, rightStepsRawPrev = 0;
		signed long int motorPositionDataCorrect[2];
		double xPos, yPos, theta;
		double deltaSteps, deltaTheta;
		rclcpp::Time currentTime, lastTime, currentTimeMap, lastTimeMap;
		int overflowCountLeft = 0, overflowCountRight = 0;

		uint8_t imu_addr = MPU9250_ADDRESS_AD1_0;
		uint8_t accData[6];
		uint8_t gyroData[6];
		uint8_t temperatureData;
		int16_t accValue[3];
		int32_t accSum[3] = {0, 0, 0};
		int16_t accOffset[3] = {0, 0, 0};
		int16_t gyroValue[3];
		int32_t gyroSum[3] = {0, 0, 0};
		int16_t gyroOffset[3] = {0, 0, 0};

		bool debug_enabled = false;
		uint8_t debug_count = 0;

		int initConnectionWithRobot(void);
		void closeConnection();
		void updateActuators();
		void mpu9250_change_addr(void);
		int read_reg(int file, uint8_t reg, int count, uint8_t *data);
		void calibrateAcc();
		void calibrateGyro();
		int update_robot_sensors_and_actuators();
		void updateSensorsData();
		void updateRosInfo();
		void handlerVelocity(const geometry_msgs::msg::Twist::ConstPtr& msg);
		void handlerLED(const std_msgs::msg::UInt8MultiArray::ConstPtr& msg);
}
