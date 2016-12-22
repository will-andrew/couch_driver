#include <iostream>
#include <unistd.h>
#include <cmath>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <termios.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

#define NUM_AXIS 6
#define COUCH_MAX 20000

using namespace std;

struct js_event {
    unsigned int time;  /* event timestamp in milliseconds */
    short value;   /* value */
    unsigned char type;     /* event type */
    unsigned char number;   /* axis/button number */
};

struct joy_status {
    int button[11];
    int axis[NUM_AXIS];
};

namespace couch_teleop
{
	class TeleopNode
	{
		public:

			TeleopNode(ros::NodeHandle nh, ros::NodeHandle pnh, int joy_fd) : 
				nh(nh), pnh(pnh)
			{
				sx = 2.5;
				sy = 2.5;
				srot = 2;
				minSpeed = 0.1;

				joystick_fd = joy_fd;

				memset(&jss, 0, sizeof(struct joy_status));
	
				cmdRawPub = nh.advertise<std_msgs::Float64MultiArray>("cmd_raw", 2);
				cmdRawMsg.data.resize(4);
    
				cout << "Starting couch driving" << endl;
			}

			~TeleopNode()
			{
				close(joystick_fd);
			}

			void spin()
			{
				// Read the joystick
				int rc = get_joystick_status(joystick_fd, &jss);
				if (rc < 0) {
					cout << "Error reading joystick" << endl;
				} else {
					// Send commands to the couch
					move_motors(&jss); 
				}
			}

		private:
			ros::NodeHandle nh;
			ros::NodeHandle pnh;
			
			ros::Publisher cmdRawPub;

			std_msgs::Float64MultiArray cmdRawMsg;

			int joystick_fd;
			struct joy_status jss;

			double sx;
			double sy;
			double srot;
			double minSpeed;

			int read_joystick_event(int joystick_fd, struct js_event *jse)
			{
				int bytes;
				bytes = read(joystick_fd, jse, sizeof(*jse)); 
				if (bytes == -1)
					return 0;

				if (bytes == sizeof(*jse))
					return 1;

				cout << "Unexpected bytes from joystick: " << bytes << endl;
				return -1;
			}

			int get_joystick_status(int joystick_fd, struct joy_status *jss)
			{
				int rc;
				struct js_event jse;
				if (joystick_fd < 0)
					return -1;

				while ((rc = read_joystick_event(joystick_fd, &jse) == 1)) {
					jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
					if (jse.type == JS_EVENT_AXIS) {
						if (jse.number < NUM_AXIS) {
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

			int cap(int a) {
				int max = COUCH_MAX;
				a = a < -max ? -max : a;
				a = a > max ? max : a;
				return a;
			} 

			double magic (double a,double b){
				return abs(a) < abs(b) ? a:b;
			}

			void move_motors(struct joy_status *js)
			{
				//Artificial deadband
				double x, y, rot;
				if (abs(js->axis[0]) < 1000) x = 0;
				else x = (double)js->axis[0] / SHRT_MAX;
				if (abs(js->axis[1]) < 1000) y = 0;
				else y = -(double)js->axis[1] / SHRT_MAX;
				if (abs(js->axis[3]) < 1000) rot = 0;
				else rot = (double)js->axis[3] / SHRT_MAX;

				x = -x;
				y = -y;
			   
				static double lfs = 0;
				static double rfs = 0;
				static double lbs = 0;
				static double rbs = 0;

				//cout << x << " " << y << " " << rot << endl;

				//Joystick cubic scaling
				double x1 = pow(x, 2) * sx;
				double y1 = pow(y, 2) * sy;
				double rot1 = pow(rot, 2) * srot; 
				if (x < 0) x1 = -x1;
				if (y < 0) y1 = -y1;
				if (rot < 0) rot1 = -rot1;
				
				//High-speed trigger.    
				if (js->axis[2] < 0) {
					x1 = x1/2;
					y1 = y1/2;
					rot1 = rot1/4;
				}else if(x1*x1+y1*y1 > 0.5 && js->axis[5] < 0){
				rot1 = rot1/4;
				}
				
				double scl = 0.99;
				double scld = 0.93;
				double lf = x1 + y1 - rot1;
				double rf = x1 - y1 + rot1;
				double lb = x1 - y1 - rot1;
				double rb = x1 + y1 + rot1;
				
				double biggest = max(abs(lf),max(abs(rf),max(abs(lb),abs(rb))));
				biggest = biggest > COUCH_MAX?biggest/COUCH_MAX:1;

				lf = lf / biggest;
				rf = rf / biggest;
				lb = lb / biggest;
				rb = rb / biggest;

				//New magic code; Fred-ified.
				lfs = magic(lfs*scl + (1-scl) * lf, lfs*scld + (1-scld) * lf);
				rfs = magic(rfs*scl + (1-scl) * rf, rfs*scld + (1-scld) * rf);
				lbs = magic(lbs*scl + (1-scl) * lb, lbs*scld + (1-scld) * lb);
				rbs = magic(rbs*scl + (1-scl) * rb, rbs*scld + (1-scld) * rb);

				//cout << "Setting couch motors to: " << lfs << " " << rfs << " " << lbs << 
				//	" " << rbs << endl;

				cmdRawMsg.data[0] = lf;
				cmdRawMsg.data[1] = lb;
				cmdRawMsg.data[2] = rf;
				cmdRawMsg.data[3] = rb;
				cmdRawPub.publish(cmdRawMsg);
			}
	};
}

#define JOY_PORT 1

int main(int argc, char **argv)
{
	ros::init(argc, argv, "couch_teleop_node");
	ros::NodeHandle nh("/teleop/");
	ros::NodeHandle pnh("~");
				
	std::string joy_port;
	pnh.param("joy_port", joy_port, std::string("/dev/input/js0"));

    cout << "opening joystick " << joy_port << endl;
    int joystick_fd = open(joy_port.c_str(), O_RDONLY | O_NONBLOCK);
    if (joystick_fd < 0) return -1;

	couch_teleop::TeleopNode *c;
	c = new couch_teleop::TeleopNode(nh, pnh, joystick_fd);
				
	ros::Rate r(50);

	while (ros::ok()) {
		c->spin();	
		ros::spinOnce();
		r.sleep();
	}

	delete c;

	return 0;
}

