
#include "epuck_driver_cpp_ros2/epuck_driver_cpp_ros2.hpp"



PiPuckRos2::PiPuckRos2() : Node("pipuck_to_ros2") {
    this->declare_parameter<float>("xpos",0.0);
    this->declare_parameter<float>("ypos",0.0);
    this->declare_parameter<float>("theta",0.0);
    this->declare_parameter<std::string>("epuck_name","epuck");
    this->declare_parameter<bool>("imu",false);
    this->declare_parameter<bool>("publish_tf",false);
    this->declare_parameter<bool>("motor_speed",false);
    this->declare_parameter<bool>("floor",false);
    this->declare_parameter<bool>("proximity",false);
    this->declare_parameter<bool>("motor_position",false);
    this->declare_parameter<bool>("microphone",false);
    this->declare_parameter<int>("ros_rate",20);
    this->declare_parameter<std::string>("epuck_name","");
    //Robot control parameters
    this->declare_parameter<int>("right_motor_speed",0);
    this->declare_parameter<int>("left_motor_speed",0);
    this->declare_parameter<int>("speaker_sound_id",0);
    this->declare_parameter<int>("normal_led",0);
    this->declare_parameter<std::vector<int64_t>>("rgb_led_2",std::vector<int64_t>{0,0,0});
    this->declare_parameter<std::vector<int64_t>>("rgb_led_4",std::vector<int64_t>{0,0,0});
    this->declare_parameter<std::vector<int64_t>>("rgb_led_6",std::vector<int64_t>{0,0,0});
    this->declare_parameter<std::vector<int64_t>>("rgb_led_8",std::vector<int64_t>{0,0,0});
    this->declare_parameter<int>("settings",0);

    //adding parameter event handler to react to parameter updates
    parameter_event_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10,
            std::bind(&PiPuckRos2::updateParameterCb, this, std::placeholders::_1)
        );
    // param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    // cb_right_motor_speed_ = param_subscriber_->add_parameter_callback("right_motor_speed",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_right_motor_speed_ = param_subscriber_->add_parameter_callback("left_motor_speed",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_speaker_sound_id_ = param_subscriber_->add_parameter_callback("speaker_sound_id",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_normal_led_ = param_subscriber_->add_parameter_callback("normal_led",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_rgb_led_2_ = param_subscriber_->add_parameter_callback("rgb_led_2",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_rgb_led_4_ = param_subscriber_->add_parameter_callback("rgb_led_4",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_rgb_led_6_ = param_subscriber_->add_parameter_callback("rgb_led_6",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_rgb_led_8_ = param_subscriber_->add_parameter_callback("rgb_led_8",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));
    // cb_settings_ = param_subscriber_->add_parameter_callback("settings",std::bind(&PiPuckRos2::updateParameterCb,this,std::placeholders::_1));

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Pubs, subs and timers
    std::string epuck_name = this->get_parameter("epuck_name").as_string();
    for( int i = 0; i < 8; i++) prox_pub_[i] = this->create_publisher<sensor_msgs::msg::Range>(epuck_name + "/proximity_sensor_" + std::to_string(i),1);
    for( int i = 0; i < 4; i++) mic_pub_[i] = this->create_publisher<std_msgs::msg::Float32>(epuck_name + "/microphone_" + std::to_string(i),1);
    motor_state_right_pub_ = this->create_publisher<std_msgs::msg::Int32>(epuck_name + "/motor_state_right",1);
    motor_state_left_pub_ = this->create_publisher<std_msgs::msg::Int32>(epuck_name + "/motor_state_left",1);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(epuck_name + "/odom",1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&PiPuckRos2::updateRobotState,this));

    //Loading values
    theta = this->get_parameter("theta").as_double();
    xPos = this->get_parameter("xPos").as_double();
    yPos = this->get_parameter("yPos").as_double();
}

PiPuckRos2::~PiPuckRos2() {
    close(fh);
}

void PiPuckRos2::updateParameterCb(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    for (const auto & changed_parameter : event->changed_parameters)   {
        if(changed_parameter.name == "right_motor_speed") {
            right_motor_speed_ = changed_parameter.integer_value;
            ros_to_epuck_[2] = right_motor_speed_&0xFF;
            ros_to_epuck_[3] = right_motor_speed_>>8;

        }
        else if(changed_parameter.name == "left_motor_speed") {
            left_motor_speed_ = changed_parameter.integer_value;
            ros_to_epuck_[0] = left_motor_speed_&0xFF;
            ros_to_epuck_[1] = left_motor_speed_>>8;
        }
        else if(changed_parameter.name == "speaker_sound_id") {
            speaker_sound_id_ = changed_parameter.integer_value;
            ros_to_epuck_[4] = speaker_sound_id_;
        }
        else if(changed_parameter.name == "normal_led") {
            normal_led_ = changed_parameter.integer_value;
            ros_to_epuck_[5] = normal_led_;
        }
        else if(changed_parameter.name == "rgb_led_2") {
            std::vector<int64_t> new_values_ = changed_parameter.integer_array_value;
            std::copy(new_values_.begin(),new_values_.end(),rgb_led_2_);
            ros_to_epuck_[6] = rgb_led_2_[0];
            ros_to_epuck_[7] = rgb_led_2_[1];
            ros_to_epuck_[8] = rgb_led_2_[2];
        }
        else if(changed_parameter.name == "rgb_led_4") {
            std::vector<int64_t> new_values_ = changed_parameter.integer_array_value;
            std::copy(new_values_.begin(),new_values_.end(),rgb_led_4_);
            ros_to_epuck_[9] = rgb_led_4_[0];
            ros_to_epuck_[10] = rgb_led_4_[1];
            ros_to_epuck_[11] = rgb_led_4_[2];
        }
        else if(changed_parameter.name == "rgb_led_6") {
            std::vector<int64_t> new_values_ = changed_parameter.integer_array_value;
            std::copy(new_values_.begin(),new_values_.end(),rgb_led_6_);
            ros_to_epuck_[12] = rgb_led_6_[0];
            ros_to_epuck_[13] = rgb_led_6_[1];
            ros_to_epuck_[14] = rgb_led_6_[2];
        }
        else if(changed_parameter.name == "rgb_led_8") {
            std::vector<int64_t> new_values_ = changed_parameter.integer_array_value;
            std::copy(new_values_.begin(),new_values_.end(),rgb_led_8_);
            ros_to_epuck_[15] = rgb_led_8_[0];
            ros_to_epuck_[16] = rgb_led_8_[1];
            ros_to_epuck_[17] = rgb_led_8_[2];
        }
        else if(changed_parameter.name == "settings") {
            settings_ = changed_parameter.integer_value;
            ros_to_epuck_[18] = settings_;
        }
    }
    return;
}

void PiPuckRos2::mpu9250_change_addr(void) {
	if(imu_addr == MPU9250_ADDRESS_AD1_0) {
		imu_addr = MPU9250_ADDRESS_AD1_1;
	} else {
		imu_addr = MPU9250_ADDRESS_AD1_0;
	}
	ioctl(fh, I2C_SLAVE, imu_addr);
}

int PiPuckRos2::read_reg(int file, uint8_t reg, int count, uint8_t *data) {
	if(write(file, &reg, 1) != 1) {
		mpu9250_change_addr();
		if(write(file, &reg, 1) != 1) {
			perror("imu write error");
			return -1;
		}
	}
	if(read(file, data, count) != count) {
		mpu9250_change_addr();
		if(read(file, data, count) != count) {
			printf("count=%d\n", count);
			perror("imu read error");
			return -1;
		}
	}
	return 0;
}

bool PiPuckRos2::initConnectionWithRobot(void) {

	// Set the I2C timeout to 20 ms (instead of 1 second). This need to be done on the "switcher" bus channel.
	int fh1 = open("/dev/i2c-1", O_RDWR);
	if(ioctl(fh1, I2C_TIMEOUT, 2) < 0) {
		perror("fail to set i2c1 timeout");
	}		
	close(fh1);

	fh = open(I2C_CHANNEL, O_RDWR);
	if(fh < 0) { // Try with bus number used in older kernel
		fh = open(LEGACY_I2C_CHANNEL, O_RDWR);	
		if(fh < 0) {
			perror("Cannot open I2C device");
			return false;
		}
	}
	return true;
}

void PiPuckRos2::publishProximityData() {
    for(int i=0; i < 8; i++) {
            sensor_msgs::msg::Range msg;
            msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
            msg.header.frame_id = "/base_prox" + std::to_string(i);
            msg.header.stamp = this->get_clock()->now();
            msg.field_of_view = 0.26;
            msg.max_range = 0.05;
            msg.min_range = 0.005;

            int proxData = epuck_to_ros_[i * 2] | epuck_to_ros_ [(i * 2) + 1] << 8;
            if(proxData > 0) {
                msg.range = 0.5/sqrt(proxData);  // Transform the analog value to a distance value in meters (given from field tests).
            } else {
                msg.range = msg.max_range;
            }
            if(msg.range > msg.max_range) {
                msg.range = msg.max_range;
            }
            if(msg.range < msg.min_range) {
                msg.range = msg.min_range;
            }
            msg.header.stamp = this->get_clock()->now();
            prox_pub_[i]->publish(msg);
        }
    if(this->get_parameter("publish_tf").as_bool()) proximityTf();
    
}

void PiPuckRos2::publishMicrophoneData() {
    int offset = 32;
    for(int i = 0; i < 4 ; i++) {
        std_msgs::msg::Float32 msg;
        msg.data = epuck_to_ros_[offset + (i * 2)] | epuck_to_ros_[offset + 1 + (i*2)] << 8;
        mic_pub_[i] ->publish(msg);
    }
    return;
}

void PiPuckRos2::publishMotorPosition() {
    std_msgs::msg::Int32 msg;
    motorPositionData[0] = epuck_to_ros_[41] | epuck_to_ros_[42] << 8;
    msg.data = motorPositionData[0];
    motor_state_left_pub_->publish(msg);
    motorPositionData[1] = epuck_to_ros_[43] | epuck_to_ros_[44] << 8;
    msg.data = motorPositionData[1];
    motor_state_right_pub_->publish(msg);
    return;
}

void PiPuckRos2::proximityTf() {

        // e-puck proximity positions (cm), x pointing forward, y pointing left
        //           P7(3.5, 1.0)   P0(3.5, -1.0)
        //       P6(2.5, 2.5)           P1(2.5, -2.5)
        //   P5(0.0, 3.0)                   P2(0.0, -3.0)
        //       P4(-3.5, 2.0)          P3(-3.5, -2.0)
        //
        // e-puck proximity orentations (degrees)
        //           P7(10)   P0(350)
        //       P6(40)           P1(320)
        //   P5(90)                   P2(270)
        //       P4(160)          P3(200)
        
        std::stringstream parent;
        std::stringstream child;
        tf2::Transform transform;
        tf2::Quaternion q;
        double rpy_data_[8] = {6.11,5.59,4.71,3.49,2.8,1.57,0.70,0.17};
        std::pair<double,double> vector_data_[8] = {{0.035,-0.010},{0.025,-0.025},{0,-0.03},{-0.035,-0.02},{-0.035,0.02},{0,0.03},{0.025,0.025},{0.035,0.01}};
       
        for(int i = 0; i < 8; i++) {
            transform.setOrigin( tf2::Vector3(vector_data_[i].first, vector_data_[i].second, 0.034) );        
            q.setRPY(0, 0, rpy_data_[i]);
            transform.setRotation(q);
            geometry_msgs::msg::TransformStamped msg;
            msg.header.frame_id = "/base_prox" + std::to_string(i);
            msg.header.stamp = this->get_clock()->now();
            msg.child_frame_id = "/base_link";
            msg.transform = tf2::toMsg(transform);
            broadcaster_->sendTransform(msg);
        }
        
        
    return;
}

void PiPuckRos2::publishImu() {

    ioctl(fh, I2C_SLAVE, imu_addr);
	
    read_reg(fh, ACCEL_XOUT_H, 6, accData);	
    read_reg(fh, GYRO_XOUT_H, 6, gyroData);		

    accValue[0] = (accData[1] + (accData[0]<<8));// MPU9250 big-endian
    accValue[1] = (accData[3] + (accData[2]<<8));
    accValue[2] = (accData[5] + (accData[4]<<8));

    gyroValue[0] = (gyroData[1] + (gyroData[0]<<8));
    gyroValue[1] = (gyroData[3] + (gyroData[2]<<8));
    gyroValue[2] = (gyroData[5] + (gyroData[4]<<8));

    // Creating msg
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = "/base_link";
    msg.header.stamp = this->get_clock()->now();           
    msg.linear_acceleration.x = (accValue[0]-accOffset[0]) * STANDARD_GRAVITY * ACC_RAW2G; // m/s^2
    msg.linear_acceleration.y = (accValue[1]-accOffset[1]) * STANDARD_GRAVITY * ACC_RAW2G;
    msg.linear_acceleration.z = (accValue[2]-accOffset[2]+GRAVITY_MPU9250) * STANDARD_GRAVITY * ACC_RAW2G;
    msg.linear_acceleration_covariance[0] = 0.01;
    msg.linear_acceleration_covariance[1] = 0.0;
    msg.linear_acceleration_covariance[2] = 0.0;
    msg.linear_acceleration_covariance[3] = 0.0;
    msg.linear_acceleration_covariance[4] = 0.01;
    msg.linear_acceleration_covariance[5] = 0.0;
    msg.linear_acceleration_covariance[6] = 0.0;
    msg.linear_acceleration_covariance[7] = 0.0;
    msg.linear_acceleration_covariance[8] = 0.01;
    msg.angular_velocity.x = (gyroValue[0] - gyroOffset[0]) * DEG2RAD(GYRO_RAW2DPS); // rad/s
    msg.angular_velocity.y = (gyroValue[1] - gyroOffset[1]) * DEG2RAD(GYRO_RAW2DPS);
    msg.angular_velocity.z = (gyroValue[2] - gyroOffset[2]) * DEG2RAD(GYRO_RAW2DPS);
    msg.angular_velocity_covariance[0] = 0.01;
    msg.angular_velocity_covariance[1] = 0.0;
    msg.angular_velocity_covariance[2] = 0.0;
    msg.angular_velocity_covariance[3] = 0.0;
    msg.angular_velocity_covariance[4] = 0.01;
    msg.angular_velocity_covariance[5] = 0.0;
    msg.angular_velocity_covariance[6] = 0.0;
    msg.angular_velocity_covariance[7] = 0.0;
    msg.angular_velocity_covariance[8] = 0.01;

    tf2::Quaternion q;
    q.setRPY(0,0,0);
    geometry_msgs::msg::Quaternion odomQuat = tf2::toMsg(q);
    msg.orientation = odomQuat;
    msg.orientation_covariance[0] = 0.01;
    msg.orientation_covariance[1] = 0.0;
    msg.orientation_covariance[2] = 0.0;
    msg.orientation_covariance[3] = 0.0;
    msg.orientation_covariance[4] = 0.01;
    msg.orientation_covariance[5] = 0.0;
    msg.orientation_covariance[6] = 0.0;
    msg.orientation_covariance[7] = 0.0;
    msg.orientation_covariance[8] = 0.01;
    imu_pub_->publish(msg);
}

void PiPuckRos2::publishOdometry() {
    // The encoders values coming from the e-puck are 2 bytes signed int thus we need to handle the overflows otherwise the odometry will be wrong after a while (about 4 meters).
    if((leftStepsRawPrev>0) && (motorPositionData[0]<0) && (abs(motorPositionData[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (positive).
        overflowCountLeft++;
    }
    if((leftStepsRawPrev<0) && (motorPositionData[0]>0) && (abs(motorPositionData[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (negative).
        overflowCountLeft--;
    }
    motorPositionDataCorrect[0] = (overflowCountLeft*65536) + motorPositionData[0];
    
    if((rightStepsRawPrev>0) && (motorPositionData[1]<0) && (abs(motorPositionData[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (positive).
        overflowCountRight++;
    }
    if((rightStepsRawPrev<0) && (motorPositionData[1]>0) && (abs(motorPositionData[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (negative).
        overflowCountRight--;
    }
    motorPositionDataCorrect[1] = (overflowCountRight*65536) + motorPositionData[1];        
    
    leftStepsRawPrev = motorPositionData[0];
    rightStepsRawPrev = motorPositionData[1];
    
    // Compute odometry.
    leftStepsDiff = motorPositionDataCorrect[0]*MOT_STEP_DIST - leftStepsPrev; // Expressed in meters.
    rightStepsDiff = motorPositionDataCorrect[1]*MOT_STEP_DIST - rightStepsPrev;   // Expressed in meters.
    
    deltaTheta = (rightStepsDiff - leftStepsDiff)/WHEEL_DISTANCE;   // Expressed in radiant.
    deltaSteps = (rightStepsDiff + leftStepsDiff)/2;        // Expressed in meters.

    xPos += deltaSteps*cos(theta + deltaTheta/2);   // Expressed in meters.
    yPos += deltaSteps*sin(theta + deltaTheta/2);   // Expressed in meters.
    theta += deltaTheta;    // Expressed in radiant.
    
    leftStepsPrev = motorPositionDataCorrect[0]*MOT_STEP_DIST;     // Expressed in meters.
    rightStepsPrev = motorPositionDataCorrect[1]*MOT_STEP_DIST;    // Expressed in meters.

    nav_msgs::msg::Odometry msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "/base_link";
    msg.pose.pose.position.x = xPos;       
    msg.pose.pose.position.y = yPos;
    msg.pose.pose.position.z = 0;
    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    tf2::Quaternion q;
    q.setRPY(0,0,theta);
    geometry_msgs::msg::Quaternion odomQuat = tf2::toMsg(q);
    msg.pose.pose.orientation = odomQuat;
    currentTime = this->get_clock()->now();
    msg.twist.twist.linear.x = deltaSteps / ((currentTime-lastTime).seconds());   // "deltaSteps" is the linear distance covered in meters from the last update (delta distance);
                                                                                    // the time from the last update is measured in seconds thus to get m/s we multiply them.
    msg.twist.twist.angular.z = deltaTheta / ((currentTime-lastTime).seconds());  // "deltaTheta" is the angular distance covered in radiant from the last update (delta angle);
                                                                                    // the time from the last update is measured in seconds thus to get rad/s we multiply them.
    lastTime = this->get_clock()->now();

    odom_pub_->publish(msg);
    
    // Publish the transform over tf.
    geometry_msgs::msg::TransformStamped odomTrans;
    odomTrans.header.stamp = msg.header.stamp;
    odomTrans.header.frame_id = msg.header.frame_id;
    odomTrans.child_frame_id = msg.child_frame_id;
    odomTrans.transform.translation.x = xPos;
    odomTrans.transform.translation.y = yPos;
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = odomQuat;
    broadcaster_->sendTransform(odomTrans);
}

void PiPuckRos2::updateRobotState() {

    //Setting up memory
    memset(epuck_to_ros_,0x0,SENSORS_SIZE);
    uint8_t checksum = 0;
	for(int i=0; i<(ACTUATORS_SIZE-1); i++) {
	    checksum ^= ros_to_epuck_[i];
    }
	ros_to_epuck_[ACTUATORS_SIZE-1] = checksum;

    if(!i2cDataExchange()) return; //data exchange failed -> not publishing new data

    //Check checksum of received data
    checksum = 0;
    for(int i=0; i<(SENSORS_SIZE-1); i++) {
    	checksum ^= epuck_to_ros_[i];
    }
    
    if(checksum == epuck_to_ros_[SENSORS_SIZE -1 ]) {
        // Publishing data if respective parameter is set to true
        if(this->get_parameter("proximity").as_bool()) publishProximityData();
        if(this->get_parameter("microphone").as_bool()) publishMicrophoneData();
        if(this->get_parameter("motor_position").as_bool()) publishMotorPosition();
        //TODO add laserscan interpolation if needed -> see old non ported code
        //TODO add ambient light values from proximity sensors if needed. Procedure same as with distance but use epuck_to_ros_ index [16] - [31]
        //TODO add state of selector button if needed (index 40). Selector + button: selector values represented by 4 least significant bits (bit0, bit1, bit2, bit3); button state is in bit4 (1=pressed, 0=not pressed)
        //TODO add tv remote if needed (index 45)    
    }

    if(this->get_parameter("imu").as_bool()) publishImu();
    if(this->get_parameter("odometry").as_bool()) publishOdometry();
    return;
}

bool PiPuckRos2::i2cDataExchange() {
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
	int trials = 0;
	
	// S Addr Wr [A] Data(actuators) NA Sr Addr Rd [A] Data(sensors) [A] P
    messages[0].addr  = ROBOT_ADDR;
    messages[0].flags = 0;
    messages[0].len   = ACTUATORS_SIZE;
    messages[0].buf   = ros_to_epuck_;
	
    messages[1].addr  = ROBOT_ADDR;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = SENSORS_SIZE;
    messages[1].buf   = epuck_to_ros_;

    packets.msgs      = messages;
    packets.nmsgs     = 2;
	
	// Form the tests it was noticed that sometimes (about 1/1000) the communication give a "timeout error" followed by "remote I/O" error.
	// Thus 3 retrials are done in case of errors.
	while(trials < 3) {
		if(ioctl(fh, I2C_RDWR, &packets) < 0) {		
			trials++;
			continue;
		}
		break;
	}

	if(trials > 2) {
		perror("update_robot_sensors_and_actuators: ");
		return false;
	} else {
		return true;
	}
}

void PiPuckRos2::calibrateAcc() {
	int samplesCount=0, i=0;
	// reset and send configuration first?
	for(i=0; i<NUM_SAMPLES_CALIBRATION; i++) {
		if(read_reg(fh, ACCEL_XOUT_H, 6, accData) == 0) {	// for MPU9250 set just the address also for a multiple read with autoincrement
			accSum[0] += (int16_t)(accData[1] + (accData[0]<<8)); // MPU9250 big-endian
			accSum[1] += (int16_t)(accData[3] + (accData[2]<<8));
			accSum[2] += (int16_t)(accData[5] + (accData[4]<<8));
			samplesCount++;
			//printf("acc sums: x=%d, y=%d, z=%d (samples=%d)\n", accSum[0], accSum[1], accSum[2], samplesCount);
		}
	}
	accOffset[0] = (int16_t)((float)accSum[0]/(float)samplesCount);
	accOffset[1] = (int16_t)((float)accSum[1]/(float)samplesCount);
	accOffset[2] = (int16_t)((float)accSum[2]/(float)samplesCount);
	RCLCPP_INFO(this->get_logger(),"acc offsets: x=%d, y=%d, z=%d (samples=%d)\n", accOffset[0], accOffset[1], accOffset[2], samplesCount);

}

void PiPuckRos2::calibrateGyro() {
	int samplesCount=0, i=0;
	// reset and send configuration first?
	for(i=0; i<NUM_SAMPLES_CALIBRATION; i++) {
		if(read_reg(fh, GYRO_XOUT_H, 6, gyroData) == 0) {	// // for MPU9250 set just the address also for a multiple read with autoincrement
			gyroSum[0] += (int16_t)(gyroData[1] + (gyroData[0]<<8)); // MPU9250 big-endian
			gyroSum[1] += (int16_t)(gyroData[3] + (gyroData[2]<<8));
			gyroSum[2] += (int16_t)(gyroData[5] + (gyroData[4]<<8));
			samplesCount++;
			//printf("gyro sums: x=%d, y=%d, z=%d (samples=%d)\n", gyroSum[0], gyroSum[1], gyroSum[2], samplesCount);
		}
	}
	gyroOffset[0] = (int16_t)((float)gyroSum[0]/(float)samplesCount);
	gyroOffset[1] = (int16_t)((float)gyroSum[1]/(float)samplesCount);
	gyroOffset[2] = (int16_t)((float)gyroSum[2]/(float)samplesCount);
	RCLCPP_INFO(this->get_logger(),"gyro offsets: x=%d, y=%d, z=%d (samples=%d)\n", gyroOffset[0], gyroOffset[1], gyroOffset[2], samplesCount);
}

int main(int argc,char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<PiPuckRos2> node = std::make_shared<PiPuckRos2>();

    if(node->initialize()) {
		rclcpp::spin(node);
    }
    else RCLCPP_ERROR(node->get_logger(),"Initialization of node failed -> shuting down.");
    
    return 0;
}



