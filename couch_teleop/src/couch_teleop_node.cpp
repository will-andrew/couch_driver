#include <unistd.h>
#include <cmath>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <stdexcept>
#include <linux/input.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

#define JS_EVENT_BUTTON 0x01 // Button pressed/released
#define JS_EVENT_AXIS 0x02 // Joystick moved
#define JS_EVENT_INIT 0x80 // Initial state of device

#define JS_NUM_AXIS 6
#define JS_VIBRATE_ID 4

using namespace std;

struct js_event {
    unsigned int time; // Event timestamp in milliseconds
    short value; // Value
    unsigned char type; // Event type
    unsigned char number; // Axis/button number
};

struct js_status {
    int button[11];
    int axis[JS_NUM_AXIS];
};

namespace couch_teleop
{
	class TeleopNode
	{
		public:

			TeleopNode()
			{
                ros::NodeHandle nh("/teleop/");
                ros::NodeHandle pnh("~");
	
                pnh.param("cmd_max", cmd_max_, 20000.0);
                pnh.param("exponent", exponent_, 1.0);
                pnh.param("enable_shutdown", enable_shutdown_, true);

                // Open device files
                ROS_INFO_STREAM("Opening joystick device");
                std::string joy_device;
                std::string joy_event_device;
                pnh.param("joy_device", joy_device, std::string("/dev/input/js0"));
                pnh.param("joy_event_device", joy_event_device, std::string("/dev/input/event12"));
                
                int joystick_fd_ = open(joy_device.c_str(), O_RDONLY | O_NONBLOCK);
                if (joystick_fd_ < 0) {
                    throw std::invalid_argument(std::string("Cannot open joystick device ") + joy_device);
                }

                int joystick_event_fd_ = open(joy_event_device.c_str(), O_RDWR | O_NONBLOCK);
                if (joystick_event_fd_ < 0) {
                    throw std::invalid_argument(std::string("Cannot open joystick event device ") + joy_event_device);
                }
	
				cmdRawPub_ = nh.advertise<std_msgs::Float64MultiArray>("cmd_raw", 2);
				cmdRawMsg_.data.resize(4);


                vibrateService_ = pnh.advertiseService("vibrate", &TeleopNode::vibrateCallback, this);
			}

			~TeleopNode()
			{
				close(joystick_fd_);
			}

			int spin(void)
			{
				// Read the joystick
                struct js_status jss;
				int rc = get_joystick_status(&jss);
				if (rc < 0) {
					ROS_ERROR_STREAM("Error reading joystick");
				} else {
                    // Check for shutdown command
                    check_shutdown(&jss);

					// Send commands to the couch
					move_motors(&jss); 
				}

                return rc;
			}

		private:
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			
			ros::Publisher cmdRawPub_;
            ros::ServiceServer vibrateService_;

			std_msgs::Float64MultiArray cmdRawMsg_;

			int joystick_fd_;
            int joystick_event_fd_;

            bool enable_shutdown_;

            double cmd_max_;
            double exponent_;

            bool vibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
            {
                vibrate();
                return true;
            }

            void vibrate(void)
            {
                struct input_event play;
                play.type = EV_FF;
                play.code = JS_VIBRATE_ID;
                play.value = 1;

                // Upload input event
                write(joystick_event_fd_, (const void*) &play, sizeof(play));
            }

            void check_shutdown(struct js_status *jss)
            {
                if (!enable_shutdown_) {
                    return;
                }

                if (jss->axes[4] > 5000 && jss->buttons[0] && jss->buttons[1]) {
                    vibrate();
                    vibrate();
                    //system("dbus-send --system --print-reply --dest=org.freedesktop.login1 /org/freedesktop/login1 \"org.freedesktop.login1.Manager.PowerOff\" boolean:true");
                }
            }

			int read_joystick_event(struct js_event *jse)
			{
				int bytes;
				bytes = read(joystick_fd_, jse, sizeof(*jse)); 
				if (bytes == -1)
					return 0;

				if (bytes == sizeof(*jse))
					return 1;

				ROS_ERROR_STREAM("Unexpected bytes from joystick: " << bytes);
				return -1;
			}

			int get_joystick_status(struct js_status *jss)
			{
                struct js_event jse;
				int rc = read_joystick_event(&jse);

				while ((rc == 1)) {
					jse.type &= ~JS_EVENT_INIT; // Ignore synthetic events
					if (jse.type == JS_EVENT_AXIS) {
						if (jse.number < JS_NUM_AXIS) {
							jss->axis[jse.number] = jse.value;
						}
					} else if (jse.type == JS_EVENT_BUTTON) {
						if (jse.number < 10 && (jse.value == 0 || jse.value == 1)) {
							jss->button[jse.number] = jse.value;
						}
					}
				}

				return 0;
			}

            // TODO part of strange filter. remove if possible
			double magic (double a, double b){
				return abs(a) < abs(b) ? a : b;
			}

			void move_motors(struct js_status *js)
			{
				double x, y, rot;
				
				// Deadband
                if (abs(js->axis[0]) < 1000) {
                    x = 0;
                } else {
                    x = (double)js->axis[0] / SHRT_MAX;
                }

				if (abs(js->axis[1]) < 1000) {
                    y = 0;
                } else {
                    y = -(double)js->axis[1] / SHRT_MAX;
                }

				if (abs(js->axis[3]) < 1000) {
                    rot = 0;
                } else {
                    rot = (double)js->axis[3] / SHRT_MAX;
                }

				x = -x;
				y = -y;

				// Exponent and scaling
				double x1 = std::pow(std::abs(x), exponent_);
                if (x < 0) {
                    x1 = -x1;
                }
				double y1 = std::pow(std::abs(y), exponent_);
                if (y < 0) {
                    y1 = -y1;
                }
				double rot1 = std::pow(std::abs(rot), exponent_);
                if (rot < 0) {
                    rot1 = -rot1;
                }
				
				// High-speed trigger.    
				if (js->axis[2] < 0) {
					x1 = x1 / 2;
					y1 = y1 / 2;
					rot1 = rot1 / 2;
				} else if (x1 * x1 + y1 * y1 > 0.5 && js->axis[5] < 0) {
				    rot1 = rot1 / 2;
				}
				
                // Mecanum kinematics
				double lf = x1 + y1 - rot1;
				double rf = x1 - y1 + rot1;
				double lb = x1 - y1 - rot1;
				double rb = x1 + y1 + rot1;
				
                // Reduce power if necessary to avoid clipping, and speed limit
				double biggest = std::max(abs(lf), std::max(abs(rf), std::max(abs(lb), std::abs(rb))));
				biggest = biggest > cmd_max_ ? (biggest / cmd_max_) : 1;

				lf = lf / biggest;
				rf = rf / biggest;
				lb = lb / biggest;
				rb = rb / biggest;

				// Some filter that Fred wrote TODO remove if possible
				static double lfs = 0;
				static double rfs = 0;
				static double lbs = 0;
				static double rbs = 0;
				double scl = 0.99;
				double scld = 0.93;
				lfs = magic(lfs * scl + (1 - scl) * lf, lfs * scld + (1 - scld) * lf);
				rfs = magic(rfs * scl + (1 - scl) * rf, rfs * scld + (1 - scld) * rf);
				lbs = magic(lbs * scl + (1 - scl) * lb, lbs * scld + (1 - scld) * lb);
				rbs = magic(rbs * scl + (1 - scl) * rb, rbs * scld + (1 - scld) * rb);

                // Pack message
				cmdRawMsg_.data[0] = lfs;
				cmdRawMsg_.data[1] = lbs;
				cmdRawMsg_.data[2] = rfs;
				cmdRawMsg_.data[3] = rbs;
				cmdRawPub_.publish(cmdRawMsg_);
			}
	};
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "couch_teleop_node");

	couch_teleop::TeleopNode *c;
    ros::Rate r(50);

    while (ros::ok()) {
        try {
            c = new couch_teleop::TeleopNode();
        } catch (std::exception& e) {
            ROS_ERROR_STREAM(e.what());
            ros::Duration(1).sleep();
            continue;
        }

        while (ros::ok()) {
            if (c->spin()) {
                break;
            }
            ros::spinOnce();
            r.sleep();
        }

        delete c;
    }

	return 0;
}

