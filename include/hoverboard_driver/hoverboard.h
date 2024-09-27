#include <ros/ros.h>
#include <boost/assign/list_of.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <string>
#include "hoverboard_driver/HoverboardConfig.h"
#include "hoverboard_driver/pid.h"
#include "protocol.h"

// TCP server
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>

class HoverboardAPI;

class Hoverboard : public hardware_interface::RobotHW {
 public:
	Hoverboard();
	~Hoverboard();

	void write(const ros::Time &time, const ros::Duration &period);
	void tick();
	void tcp_server();
	void close_tcp_server();
 private:

	void on_hoverboard_data();
	void on_hoverboard_state_changed(bool state);
	void on_encoder_update(int16_t right, int16_t left);
	void publish_bool(ros::Publisher pub, bool state);
	void on_tcp_data(char byte);
	double map(double x, double in_min, double in_max, double out_min, double out_max);

	hardware_interface::JointStateInterface joint_state_interface;
	hardware_interface::VelocityJointInterface velocity_joint_interface;

	// The units for wheels are radians (pos), radians per second (vel,cmd), and Newton metres (eff)
	struct Joint {
		std_msgs::Float64 pos;
		std_msgs::Float64 vel;
		std_msgs::Float64 eff;
		std_msgs::Float64 cmd;
	} joints[2];

	// Publishers
	ros::NodeHandle nh;
	ros::Publisher vel_pub[2];
	ros::Publisher pos_pub[2];
	ros::Publisher cmd_pub[2];
	ros::Publisher voltage_pub;
	ros::Publisher temp_pub;
	ros::Publisher hb_set_speed_pub;
	ros::Publisher hb_connected_pub;
	ros::Publisher hb_left_pid;
	ros::Publisher hb_right_pid;

	// Subscribers
	ros::Subscriber hb_raw_data;

	// TCP server
	void start_tcp_server();
	int connFd = -1;
	int serverFd = -1;

	double wheel_radius;
	double max_velocity = 0.0;
	int direction_correction = 1;

	// Last time read message from hoverboard
	ros::Time last_read_hb;
	// Last known encoder values
	int16_t last_wheelcountR;
	int16_t last_wheelcountL;
	// Count of full encoder wraps
	int multR;
	int multL;
	// Thresholds for calculating the wrap
	int low_wrap;
	int high_wrap;
	// Received speed zeros
	int left_speed_zeros_count = 0;
	int right_speed_zeros_count = 0;

	float prevSL = 0;
	float prevSR = 0;

	bool tcp_server_running = true;

	// UART
	char *p;
	int port_fd;
	int msg_len = 0;
	char prev_byte = 0;
	uint16_t start_frame = 0;
	uint16_t current_start_frame = 0;

	// Hoverboard protocol
	SerialFeedback hb_msg;
	std::size_t serialFeedbackSize;

	PID pids[2];
};
