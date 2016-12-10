#include "Couch.h"
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <termios.h>
#include <fcntl.h>

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


Couch *steve;
double sx, sy, srot, minSpeed;

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


static void printStatus(struct status *s) {
    cout << "Battery voltage:" <<  s->battery_voltage << endl;
}

static void printFault(struct fault *f) {
    cout << "Fault code: " << (char)f->fault_code << " source " << 
        f->fault_source << " max " << f->max_fault_val << endl;
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
   
    static double lfs = 0;
    static double rfs = 0;
    static double lbs = 0;
    static double rbs = 0;

    cout << x << " " << y << " " << rot << endl;

    //old parabolic scaling code
    //double xDir = x > 0 ? 1 : x < 0 ? -1 : 0;
    //double x1 = (pow(x, 2) * (1.0 - minSpeed) * xDir ) * sx + minSpeed * xDir * 0.9;
    //double yDir = y > 0 ? 1 : y < 0 ? -1 : 0;
    //double y1 = (pow(y, 2) * (1.0 - minSpeed) * yDir) * sy  + minSpeed * yDir * 0.9;
    //double zDir = rot > 0 ? 1 : rot < 0 ? -1 : 0;
    //double rot1 = (pow(rot, 2) * (1.0 - minSpeed) * zDir) * srot + minSpeed * zDir;

    //Joystick cubic scaling
    double x1 = pow(x, 3) * sx;
    double y1 = pow(y, 3) * sy;
    double rot1 = pow(rot, 3) * srot; 
    
    //High-speed trigger.    
    if (js->axis[2] < 0) {
        x1 = x1/2;
        y1 = y1/2;
        rot1 = rot1/4;
    }else if(x1*x1+y1*y1 > 0.5 && js->axis[5] < 0){
	rot1 = rot1/4;
    }
    
    //Rotataion/translation scaling:
   // x1 = x1*0.8; y1 = y1*0.8;
    
     
    double scl = 0.99;
    double scld = 0.93;
  //Old magic code for backup purposes
  //  lfs = magic(lfs*scl + (1-scl)*cap(x1 + y1 + rot1),(double)cap(x1 + y1 + rot1));
  //  rfs = magic(rfs*scl + (1-scl)*cap(x1 - y1 + rot1),(double)cap(x1 - y1 + rot1));
  //  lbs = magic(lbs*scl + (1-scl)*cap(x1 - y1 - rot1),(double)cap(x1 - y1 - rot1));
  //  rbs = magic(rbs*scl + (1-scl)*cap(x1 + y1 - rot1),(double)cap(x1 + y1 - rot1));

    double lf = x1 + y1 + rot1;
    double rf = x1 - y1 + rot1;
    double lb = x1 - y1 - rot1;
    double rb = x1 + y1 - rot1;
    
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

    cout << "Setting couch motors to: " << lfs << " " << rfs << " " << lbs << 
        " " << rbs << endl;
    steve->setMotors(lfs, rfs, lbs, rbs);
}

// Input argument index
#define COUCH_PORT 1
#define JOY_PORT 2

int main(int argc, char **argv)
{
    steve = new Couch(argv[COUCH_PORT]);
    cout << "opening joystick " << argv[joy_port] << endl;
    int joystick_fd = open(argv[joy_port], o_rdonly | o_nonblock);
    if (joystick_fd < 0) return -1;

    sx = 32000.0;
    sy = 32000.0;
    srot = 30000.0;
    minSpeed = 0.1;

    struct joy_status jss;
    memset(&jss, 0, sizeof(struct joy_status));

    cout << "Starting couch driving" << endl;
    while (true) { // probably should change this to exit on key press
        // Publish status message
        /*struct status *s = steve->getStatus();
          printStatus(s);
          struct fault *f = steve->getFault();
          printFault(f);
          delete s;
          delete f;*/
        // Read the joystick
        int rc = get_joystick_status(joystick_fd, &jss);
        if (rc < 0) {
            cout << "Error reading joystick" << endl;
            continue;
        }

        // Send commands to the couch
        move_motors(&jss); 
	usleep(10000);
    }

    close(joystick_fd);
    //delete steve;
    return 0;
}

