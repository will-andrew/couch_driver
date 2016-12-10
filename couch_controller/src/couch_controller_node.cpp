#include <stdint.h>
#include <ctime>
#include <sys/time.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

#include "couch_controller/couch_controller.h"
#include "couch_controller/message.h" //TODO only required because of timestamp struct. this should not be necessary

namespace couch_controller
{
	class ControllerNode
	{
		public:
			static const double CMDVEL_TIMEOUT_RES = 0.1;

			ControllerNode(ros::NodeHandle nh, ros::NodeHandle pnh) : 
				nh(nh), pnh(pnh)
			{
				odomRawPub = nh.advertise<std_msgs::Float64MultiArray>("odom_raw", 2);
				estopPub = nh.advertise<std_msgs::Bool>("estop", 2);
				vbatPub = nh.advertise<std_msgs::Float64>("battery", 2);
				mtempsPub = nh.advertise<std_msgs::Int32MultiArray>("mtemps", 2);
				ctempsPub = nh.advertise<std_msgs::Int32MultiArray>("ctemps", 2);
				currentsPub = nh.advertise<std_msgs::Float64MultiArray>("currents", 2);
				
				cmdVelSub = nh.subscribe("cmd_vel", 1, &ControllerNode::cmdVelCallback,
						this, ros::TransportHints().tcpNoDelay());
				cmdRawSub = nh.subscribe("cmd_raw", 1, &ControllerNode::cmdRawCallback,
						this, ros::TransportHints().tcpNoDelay());

				std::string ip;
				pnh.param("ip_addr", ip, std::string("192.168.1.185"));
				controller = new Controller(ip);

				setTime();

				/*double lp, li, ld, lffp, lffd, lffdd, rp, ri, rd, rffp, rffd, rffdd;
				pnh.param("lp", lp, 0.30);
				pnh.param("li", li, 0.03);
				pnh.param("ld", ld, 0.01);
				pnh.param("lffp", lffp, 0.0);
				pnh.param("lffd", lffd, 0.0);
				pnh.param("lffdd", lffdd, 0.0);

				pnh.param("rp", rp, 0.30);
				pnh.param("ri", ri, 0.03);
				pnh.param("rd", rd, 0.01);
				pnh.param("rffp", rffp, 0.0);
				pnh.param("rffd", rffd, 0.0);
				pnh.param("rffdd", rffdd, 0.0);
				controller->setPIDGains(lp, li, ld, lffp, lffd, lffdd, rp, ri, rd, rffp, rffd, rffdd);
				*/

				int decimation;
				pnh.param("vbat_estop_temp_decimation", decimation, 10);
				if (decimation < 1) { decimation = 1; }
				if (decimation > 1000) { decimation = 1000; }
				vbatEstopDecimation = decimation;

				int hz;
				pnh.param("publish_hz", hz, 50);
				if (hz < 1) { hz = 1; }
				if (hz > 200) {hz = 200; }
				statusHz = hz;
				controller->setStatusHz((uint8_t)hz);

				double c;
				pnh.param("cmdvel_timeout", c, 0.5);
				cmdVelTimeout = (int) (c / CMDVEL_TIMEOUT_RES);
				cmdVelTimeoutTimer = nh.createTimer(ros::Duration(CMDVEL_TIMEOUT_RES),
							&ControllerNode::cmdVelTimeoutCallback, this);

				pnh.param("max_linear", maxLinear, 4.0);
				pnh.param("max_angular", maxAngular, 6.0);
				
				odomRawMsg.data.resize(8);
				ctempsMsg.data.resize(4);
				mtempsMsg.data.resize(4);
				currentsMsg.data.resize(4);

				estopState = false;
				cmdVelTimeoutCounter = 0;
				spinCounter = 0;
			}

			~ControllerNode()
			{
				delete controller;
			}

			void spin()
			{
				struct controllerStatus status;
				try {
					status = controller->getStatusBlocking();
					updateStatus(status);
				} catch (std::exception& ex) {
					controller->setStatusHz(statusHz);
					ROS_ERROR_STREAM(ex.what());
				}
				
				ros::spinOnce();
			}

		private:
			Controller *controller;

			ros::NodeHandle nh;
			ros::NodeHandle pnh;
			
			ros::Publisher odomRawPub;
			ros::Publisher estopPub;
			ros::Publisher vbatPub;
			ros::Publisher mtempsPub;
			ros::Publisher ctempsPub;
			ros::Publisher currentsPub;

			ros::Subscriber cmdVelSub;
			ros::Subscriber cmdRawSub;

			ros::Timer cmdVelTimeoutTimer;

			unsigned int vbatEstopDecimation;

			bool estopState;

			std_msgs::Float64MultiArray odomRawMsg;
			std_msgs::Int32MultiArray ctempsMsg;
			std_msgs::Int32MultiArray mtempsMsg;
			std_msgs::Float64MultiArray currentsMsg;
			
			double maxLinear;
			double maxAngular;

			int spinCounter;

			enum safety {
				SAFETY_OFF = 0,
				SAFETY_ON = 1,
				SAFETY_FLASHING = 2
			} safetyState;

			int cmdVelTimeoutCounter;
			int cmdVelTimeout;
				  
			int statusHz;

			void setTime()
			{
				ros::Time rostime = ros::Time::now();
				struct timestamp time;
				time.tv_sec = rostime.sec;
				time.tv_nsec = rostime.nsec;
				controller->setTime(time);
			}

			void cmdVelTimeoutCallback(const ros::TimerEvent&)
			{
				if (cmdVelTimeoutCounter > cmdVelTimeout) {
					// No new cmd_vel message for a while
					double vels[4] = {0, 0, 0, 0};
					controller->setVels(vels);

				if (cmdVelTimeoutCounter == (cmdVelTimeout+1))
					ROS_WARN("cmd_vel timeout occurred\n");
				}

				cmdVelTimeoutCounter++;
			}

			void updateStatus(const struct controllerStatus& status)
			{
				static int decimateCounter = 0;
				if (++decimateCounter >= vbatEstopDecimation) {
					decimateCounter = 0;
					
					std_msgs::Float64 vbatMsg;
					vbatMsg.data = status.vBat;
					vbatPub.publish(vbatMsg);

					std_msgs::Bool estopMsg;
					estopMsg.data = status.estop;
					estopState = status.estop;
					estopPub.publish(estopMsg);

					mtempsMsg.data[0] = status.mtemps[Controller::FRONT_LEFT];
					mtempsMsg.data[1] = status.mtemps[Controller::BACK_LEFT];
					mtempsMsg.data[2] = status.mtemps[Controller::FRONT_RIGHT];
					mtempsMsg.data[3] = status.mtemps[Controller::BACK_RIGHT];
					mtempsPub.publish(mtempsMsg);
					
					ctempsMsg.data[0] = status.ctemps[Controller::FRONT_LEFT];
					ctempsMsg.data[1] = status.ctemps[Controller::BACK_LEFT];
					ctempsMsg.data[2] = status.ctemps[Controller::FRONT_RIGHT];
					ctempsMsg.data[3] = status.ctemps[Controller::BACK_RIGHT];
					ctempsPub.publish(ctempsMsg);
				}

				ros::Time rosTimestamp(status.timestamp.tv_sec, status.timestamp.tv_nsec);

				ros::Time now = ros::Time::now();
				// fallback time calculation if time is bad
				if (fabs((now - rosTimestamp).toSec()) > 0.05) {
					std::cout << "rt " << rosTimestamp << ", now " << now << std::endl;
					setTime();
					rosTimestamp = now;
				}

				currentsMsg.data[0] = status.currents[Controller::FRONT_LEFT];
				currentsMsg.data[1] = status.currents[Controller::BACK_LEFT];
				currentsMsg.data[2] = status.currents[Controller::FRONT_RIGHT];
				currentsMsg.data[3] = status.currents[Controller::BACK_RIGHT];
				currentsPub.publish(currentsMsg);

				// broadcast odom
				odomRawMsg.data[0] = status.disps[Controller::FRONT_LEFT];
				odomRawMsg.data[1] = status.disps[Controller::BACK_LEFT];
				odomRawMsg.data[2] = status.disps[Controller::FRONT_RIGHT];
				odomRawMsg.data[3] = status.disps[Controller::BACK_RIGHT];
				odomRawMsg.data[4] = status.vels[Controller::FRONT_LEFT];
				odomRawMsg.data[5] = status.vels[Controller::BACK_LEFT];
				odomRawMsg.data[6] = status.vels[Controller::FRONT_RIGHT];
				odomRawMsg.data[7] = status.vels[Controller::BACK_RIGHT];
				odomRawPub.publish(odomRawMsg);
			}

			void processCmdVel(double (&vels)[4], const geometry_msgs::Twist& cmdVel)
			{
				geometry_msgs::Twist cmd = cmdVel;

				if (cmd.linear.x > maxLinear)
					cmd.linear.x = maxLinear;
				else if (cmd.linear.x < -maxLinear)
					cmd.linear.x = -maxLinear;

				if (cmd.linear.y > maxLinear)
					cmd.linear.y = maxLinear;
				else if (cmd.linear.y < -maxLinear)
					cmd.linear.y = -maxLinear;

				if (cmd.angular.z > maxAngular)
					cmd.angular.z = maxAngular;
				else if (cmd.angular.z < -maxAngular)
					cmd.angular.z = -maxAngular;

//				if (estopState) {
					double rot = cmd.angular.z * (Controller::BASE_WIDTH / 2 + Controller::BASE_LENGTH / 2);
					vels[Controller::FRONT_LEFT] = cmd.linear.x + cmd.linear.y + rot;
					vels[Controller::BACK_LEFT] = cmd.linear.x - cmd.linear.y - rot;
					vels[Controller::FRONT_RIGHT] = cmd.linear.x - cmd.linear.y + rot;
					vels[Controller::BACK_RIGHT] = cmd.linear.x + cmd.linear.y - rot;
//				} else {
//					for (int i = 0; i < 4; i++) {
//						vels[i] = 0;
//					}
//				}
//
			}

			void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
			{
				cmdVelTimeoutCounter = 0;

				double vels[4];
				processCmdVel(vels, *msg);
				controller->setVels(vels);
			}

			void cmdRawCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
			{
				cmdVelTimeoutCounter = 0;

				double vels[4];
				
				vels[Controller::FRONT_LEFT] = msg->data[0];
				vels[Controller::FRONT_RIGHT] = msg->data[1];
				vels[Controller::BACK_LEFT] = msg->data[2];
				vels[Controller::BACK_RIGHT] = msg->data[3];

				for (int i = 0; i < 4; i++) {
					if (vels[i] > 1) {
						vels[i] = 1;
					} else if (vels[i] < -1) {
						vels[i] = -1;
					}

				}
ROS_INFO_STREAM(vels[0]);
				controller->setVels(vels);
			}
	};
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "couch_controller_node");
	ros::NodeHandle nh("controller");
	ros::NodeHandle pnh("~");

	couch_controller::ControllerNode *c;

	ros::Rate startupRetry(1);
	while (ros::ok()) {
		try {
			c = new couch_controller::ControllerNode(nh, pnh);
			break;
		} catch (std::exception& ex) {
			ROS_ERROR_STREAM(ex.what() << " retrying in 1 second...");
		}
	
		ros::spinOnce();
		startupRetry.sleep();
	}

	ROS_INFO_STREAM("Connected to controller!");

	while (ros::ok()) {
		try {
			c->spin();
		} catch (std::exception& ex) {
			ROS_WARN_STREAM(ex.what());
		}
	}

	delete c;

	return 0;
}

