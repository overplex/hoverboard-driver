#include "config.h"
#include "hoverboard.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dynamic_reconfigure/server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

Hoverboard::Hoverboard() {

	serialFeedbackSize = sizeof(SerialFeedback);

	ros::V_string joint_names = boost::assign::list_of
		("front_left_wheel_joint")("front_right_wheel_joint")
		("rear_left_wheel_joint")("rear_right_wheel_joint");

	for (unsigned int i = 0; i < joint_names.size(); i++) {

		hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
																&joints[i].pos.data,
																&joints[i].vel.data,
																&joints[i].eff.data);
		joint_state_interface.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(joint_state_handle, &joints[i].cmd.data);
		velocity_joint_interface.registerHandle(joint_handle);
	}
	registerInterface(&joint_state_interface);
	registerInterface(&velocity_joint_interface);

	// Publishers
	vel_pub[0] = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/velocity", 3);
	vel_pub[1] = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/velocity", 3);
	pos_pub[0] = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/position", 3);
	pos_pub[1] = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/position", 3);
	cmd_pub[0] = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/cmd", 3);
	cmd_pub[1] = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/cmd", 3);
	voltage_pub = nh.advertise<std_msgs::Float32>("hoverboard/battery_voltage", 3);
	temp_pub = nh.advertise<sensor_msgs::Temperature>("hoverboard/temperature", 3);
	hb_connected_pub = nh.advertise<std_msgs::Bool>("hoverboard/connected", 3);
	hb_left_pid = nh.advertise<std_msgs::Int16>("hoverboard/left_wheel/pid", 3);
	hb_right_pid = nh.advertise<std_msgs::Int16>("hoverboard/right_wheel/pid", 3);

	std::size_t error = 0;
	error +=
		!rosparam_shortcuts::get("hoverboard_driver", nh, "hoverboard_velocity_controller/wheel_radius", wheel_radius);
	error += !rosparam_shortcuts::get("hoverboard_driver",
									  nh,
									  "hoverboard_velocity_controller/linear/x/max_velocity",
									  max_velocity);
	rosparam_shortcuts::shutdownIfError("hoverboard_driver", error);

	// Convert m/s to rad/s
	max_velocity /= wheel_radius;

	low_wrap = ENCODER_LOW_WRAP_FACTOR * (ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
	high_wrap = ENCODER_HIGH_WRAP_FACTOR * (ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
	last_wheelcountR = last_wheelcountL = 0;
	multR = multL = 0;

	ros::NodeHandle nh_left(nh, "pid/left");
	ros::NodeHandle nh_right(nh, "pid/right");

	// Init PID controller
	pids[0].init(nh_left, 0.2, 0.05, 0.1, 0.0, max_velocity, -max_velocity, true, max_velocity, -max_velocity);
	pids[0].setOutputLimits(-max_velocity, max_velocity);
	pids[1].init(nh_right, 0.2, 0.05, 0.1, 0.0, max_velocity, -max_velocity, true, max_velocity, -max_velocity);
	pids[1].setOutputLimits(-max_velocity, max_velocity);

	start_tcp_server();
}

Hoverboard::~Hoverboard() {
	close_tcp_server();
	printf("[ Hoverboard ] Finish\n");
}

void Hoverboard::close_tcp_server() {
	tcp_server_running = false;
	shutdown(serverFd, 1);
}

void Hoverboard::on_hoverboard_state_changed(bool state) {
	publish_bool(hb_connected_pub, state);
	// TODO publish_bool(hb_left_pid_enable, state);
	//publish_bool(hb_right_pid_enable, state);
}

void Hoverboard::publish_bool(ros::Publisher pub, bool state) {
	std_msgs::Bool b;
	b.data = state;
	pub.publish(b);
}

void Hoverboard::on_hoverboard_data() {

	uint16_t checksum = (uint16_t)(
		hb_msg.start ^
			//hb_msg.cmd1 ^
			//hb_msg.cmd2 ^
			hb_msg.leftSpeed ^
			hb_msg.rightSpeed ^
			hb_msg.leftTicks ^
			hb_msg.rightTicks ^
			hb_msg.batVoltage ^
			hb_msg.boardTemp);

	if (hb_msg.checksum == checksum) {

		last_read_hb = ros::Time::now();

		std_msgs::Float32 f;
		f.data = (double)hb_msg.batVoltage / 100.0;
		voltage_pub.publish(f);

		sensor_msgs::Temperature temp;
		temp.header.stamp = ros::Time::now();
		temp.temperature = (double)hb_msg.boardTemp / 10.0;
		temp_pub.publish(temp);

		// Ignoring sudden speed drops to zero for left wheel
		if (hb_msg.leftSpeed != 0 || left_speed_zeros_count == 15) {
			left_speed_zeros_count = 0;
			// Convert RPM to RAD/S
			joints[0].vel.data = hb_msg.leftSpeed * 0.10472;
			// Rear left wheel
			joints[2].vel.data = joints[0].vel.data;
			// Publish left wheel speed
			vel_pub[0].publish(joints[0].vel);
		} else {
			left_speed_zeros_count++;
		}

		// Ignoring sudden speed drops to zero for right wheel
		if (hb_msg.rightSpeed != 0 || right_speed_zeros_count == 15) {
			right_speed_zeros_count = 0;
			// Convert RPM to RAD/S
			joints[1].vel.data = hb_msg.rightSpeed * -0.10472;
			// Rear right wheel
			joints[3].vel.data = joints[1].vel.data;
			// Publish right wheel speed
			vel_pub[1].publish(joints[1].vel);
		} else {
			right_speed_zeros_count++;
		}

		// Process encoder values and update odometry
		on_encoder_update(hb_msg.rightTicks, hb_msg.leftTicks);
	}
}

void Hoverboard::on_tcp_data(char byte) {
	start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

	// Read the start frame
	if (start_frame == HB_START_FRAME) {
		p = (char *)&hb_msg;
		*p++ = prev_byte;
		*p++ = byte;
		msg_len = 2;
		current_start_frame = start_frame;
	} else if (msg_len >= 2) {
		// Otherwise just read the message content until the end
		if (current_start_frame == HB_START_FRAME && msg_len < serialFeedbackSize) {
			*p++ = byte;
			msg_len++;
		}
	}

	if (msg_len == serialFeedbackSize) {
		on_hoverboard_data();
		msg_len = 0;
		current_start_frame = 0;
	}

	prev_byte = byte;
}

double Hoverboard::map(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Hoverboard::write(const ros::Time &time, const ros::Duration &period) {

	if ((ros::Time::now() - last_read_hb).toSec() > 1) {
		on_hoverboard_state_changed(false);
	} else {
		on_hoverboard_state_changed(true);
	}

	// Inform interested parties about the commands we've got
	cmd_pub[0].publish(joints[0].cmd);
	cmd_pub[1].publish(joints[1].cmd);

	double pid_outputs[2];
	pid_outputs[0] = pids[0](joints[0].vel.data, joints[0].cmd.data, period);
	pid_outputs[1] = pids[1](joints[1].vel.data, joints[1].cmd.data, period);

	std_msgs::Int16 i;

	int left_wheel_pid = map(pid_outputs[0], -max_velocity, max_velocity, 0, 4095);
	i.data = left_wheel_pid;
	hb_left_pid.publish(i);

	int right_wheel_pid = map(pid_outputs[1], -max_velocity, max_velocity, 0, 4095);
	i.data = right_wheel_pid;
	hb_right_pid.publish(i);

	/*if (joints[0].cmd.data != 0 || prevSL != 0 ||
		joints[1].cmd.data != 0 || prevSR != 0) {
		ROS_INFO("Set speed: %f %f PID: %d %d",
				 joints[0].cmd.data, joints[1].cmd.data,
				 left_wheel_pid, right_wheel_pid);
		prevSL = joints[0].cmd.data;
		prevSR = joints[1].cmd.data;
	}*/

	// 1-я проблема была в интерфейсе React (покачивание робота) постоянная отправка команды движения
	// Ограничил скорость до 300
	// При отключении гироскутера передается теперь состояние для pid_enable и обнуляются значения pid
	// TODO сброс скорости и позиции при отключении hoverboard ()
}

void Hoverboard::on_encoder_update(int16_t right, int16_t left) {
	double posL = 0.0, posR = 0.0;

	// Calculate wheel position in ticks, factoring in encoder wraps
	if (right < low_wrap && last_wheelcountR > high_wrap)
		multR++;
	else if (right > high_wrap && last_wheelcountR < low_wrap)
		multR--;
	posR = right + multR * (ENCODER_MAX - ENCODER_MIN);
	last_wheelcountR = right;

	if (left < low_wrap && last_wheelcountL > high_wrap)
		multL++;
	else if (left > high_wrap && last_wheelcountL < low_wrap)
		multL--;
	posL = left + multL * (ENCODER_MAX - ENCODER_MIN);
	last_wheelcountL = left;

	// When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddenly lost
	// This section accumulates ticks even if board shuts down and is restarted
	static double lastPosL = 0.0, lastPosR = 0.0;
	static double lastPubPosL = 0.0, lastPubPosR = 0.0;
	static bool nodeStartFlag = true;

	//IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restarted
	//(the board seems to often report 1-3 ticks on startup instead of zero)
	//reset the last read ticks to the startup values
	if ((ros::Time::now() - last_read_hb).toSec() > 0.2
		&& abs(posL) < 5 && abs(posR) < 5) {
		lastPosL = posL;
		lastPosR = posR;
	}
	double posLDiff = 0;
	double posRDiff = 0;

	//if node is just starting keep odom at zeros
	if (nodeStartFlag) {
		nodeStartFlag = false;
	} else {
		posLDiff = posL - lastPosL;
		posRDiff = posR - lastPosR;
	}

	lastPubPosL += posLDiff;
	lastPubPosR += posRDiff;
	lastPosL = posL;
	lastPosR = posR;

	// Convert position in accumulated ticks to position in radians
	joints[0].pos.data = 2.0 * M_PI * lastPubPosL / (double)TICKS_PER_ROTATION;
	joints[1].pos.data = 2.0 * M_PI * lastPubPosR / (double)TICKS_PER_ROTATION;

	// Rear wheels
	joints[2].pos.data = joints[0].pos.data;
	joints[3].pos.data = joints[1].pos.data;

	pos_pub[0].publish(joints[0].pos);
	pos_pub[1].publish(joints[1].pos);
}

void Hoverboard::tcp_server() {

	sockaddr_in serverAddr;

	if ((serverFd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		ROS_FATAL("Failed to create TCP socket");
		return;
	}

	//fcntl(serverFd, F_SETFL, O_NONBLOCK);
	memset((sockaddr*)&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET; // IPv4 address family
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY); // Local address
	serverAddr.sin_port = htons(9050); // Local port

	struct timeval tv;
	tv.tv_sec = 1000;
	tv.tv_usec = 0;
	setsockopt(serverFd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

	const int enable = 1;
	setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));

	if (bind(serverFd, (sockaddr*)&serverAddr, sizeof(serverAddr)) != 0) {
		ROS_FATAL("Failed to bind TCP socket");
		return;
	}

	if ((listen(serverFd, 1)) != 0) {
		ROS_FATAL("Failed to start TCP listening");
		return;
	}

	const int BUFFER_SIZE = 16;
	unsigned char buffer[BUFFER_SIZE];
	sockaddr_in clientAddr;
	socklen_t clientAddrLen = sizeof(clientAddr);

	while (tcp_server_running) {
		if (connFd >= 0) {
			if (recv(connFd, &buffer, BUFFER_SIZE, 0) > 0) {
				for (int i = 0; i < BUFFER_SIZE; i++) {
					on_tcp_data(buffer[i]);
				}
			} else {
				connFd = -1;
				ROS_INFO("TCP client disconnected");
			}
		} else {
			connFd = accept(serverFd, (sockaddr*)&clientAddr, &clientAddrLen);
			if (connFd >= 0) {
				ROS_INFO("TCP client connected");
			}
		}
	}

	close(serverFd);
	printf("TCP sever finished\n");
}

void* tcp_server_thread(void* context) {
	((Hoverboard*)context)->tcp_server();
	return NULL;
}

void Hoverboard::start_tcp_server() {
	pthread_t tcp_thread;
	pthread_create(&tcp_thread, NULL, tcp_server_thread, this);
	//(void) pthread_join(tcp_thread, NULL);
}
