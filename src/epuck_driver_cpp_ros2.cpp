
#include "epuck_driver_cpp_ros2.hpp"



PiPuckRos2::PiPuckRos2() : Node("pipuck_to_ros2") {
    this->declare_parameter<float>("xpos",0.0);
    this->declare_parameter<float>("ypos",0.0);
    this->declare_parameter<float>("theta",0.0);
    this->declare_parameter<std::string>("epuck_name","epuck");
    this->declare_parameter<bool>("imu",false);
    this->declare_parameter<bool>("motor_speed",false);
    this->declare_parameter<bool>("floor",false);
    this->declare_parameter<bool>("proximity",false);
    this->declare_parameter<bool>("motor_position",false);
    this->declare_parameter<bool>("microphone",false);
    this->declare_parameter<bool>("debug",false);
    this->declare_parameter<int>("ros_rate",20);

    // Pubs, subs and timers
    for( int i = 0; i < 8; i++) proxPublisher[i] = this->create_publisher<sensor_msgs::msg::Range>("/proximity_sensor_" + std::to_string(i),1);
    laserPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/tof_sensor",1);
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom",1);
    
}

PiPuckRos2::~PiPuckRos2() {
    close(fh);
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

int main(int argc,char *argv[]) {
   //TODO init connection with robot with namespace?
   double init_xpos, init_ypos, init_theta;   
   int rosRate = 0;
   int i = 0;
   
   	zero_to_epuck_buff[4] = 0; // Speaker => 0 = no sound.
   
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    rclcpp::init(argc, argv);
    
    np.param("imu", enabledSensors[IMU], false);
    np.param("motor_speed", enabledSensors[MOTOR_SPEED], false);
    np.param("floor", enabledSensors[FLOOR], false);
    np.param("proximity", enabledSensors[PROXIMITY], false);
    np.param("motor_position", enabledSensors[MOTOR_POSITION], false);
    np.param("microphone", enabledSensors[MICROPHONE], false);
	np.param("debug", debug_enabled, false);
    

    if(initConnectionWithRobot()<0) {
		return -1;
    }
    
    if(enabledSensors[IMU]) {
		ioctl(fh, I2C_SLAVE, imu_addr);	
		calibrateAcc();
		calibrateGyro();
        imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 10);
    }
    if(enabledSensors[MOTOR_SPEED]) {
        motorSpeedPublisher = n.advertise<visualization_msgs::Marker>("motor_speed", 10);
    }
    if(enabledSensors[FLOOR]) {
        floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
    }
    if(enabledSensors[PROXIMITY]) {
        for(i=0; i<8; i++) {
            /**
            * The advertise() function is how you tell ROS that you want to
            * publish on a given topic name. This invokes a call to the ROS
            * master node, which keeps a registry of who is publishing and who
            * is subscribing. After this advertise() call is made, the master
            * node will notify anyone who is trying to subscribe to this topic name,
            * and they will in turn negotiate a peer-to-peer connection with this
            * node.  advertise() returns a Publisher object which allows you to
            * publish messages on that topic through a call to publish().  Once
            * all copies of the returned Publisher object are destroyed, the topic
            * will be automatically unadvertised.
            *
            * The second parameter to advertise() is the size of the message queue
            * used for publishing messages.  If messages are published more quickly
            * than we can send them, the number here specifies how many messages to
            * buffer up before throwing some away.
            */
            std::stringstream ss;
            ss.str("");
            ss << "proximity" << i;
            proxPublisher[i] = n.advertise<sensor_msgs::Range>(ss.str(), 10);
            //proxMsg[i] = new sensor_msgs::Range();
            proxMsg[i].radiation_type = sensor_msgs::Range::INFRARED;
            ss.str("");
            ss << epuckname << "/base_prox" << i;
            proxMsg[i].header.frame_id =  ss.str();
            proxMsg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
            proxMsg[i].min_range = 0.005;       // 0.5 cm.
            proxMsg[i].max_range = 0.05;        // 5 cm.                    
        }       
        
        laserPublisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);
    }
    if(enabledSensors[MOTOR_POSITION]) {
        odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
        currentTime = ros::Time::now();
        lastTime = ros::Time::now();        
    }
    if(enabledSensors[MICROPHONE]) {
        microphonePublisher = n.advertise<visualization_msgs::Marker>("microphone", 10);
    }
       
    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called handlerVelocity.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);
    cmdLedSubscriber = n.subscribe("mobile_base/cmd_led", 10, handlerLED);
   
    theta = init_theta;
    xPos = init_xpos;
    yPos = init_ypos;

    ros::Rate loop_rate(rosRate);
   
    while (ros::ok()) {
        updateSensorsData();
        updateRosInfo();
        updateActuators();
        ros::spinOnce();
        loop_rate.sleep();    // Do not call "sleep" otherwise the bluetooth communication will hang.
                                // We communicate as fast as possible, this shouldn't be a problem...
        if(consecutiveReadTimeout >= MAX_CONSECUTIVE_TIMEOUT) { // We have connection problems, stop here.
            break;
        }
    }

    closeConnection();
    
}



