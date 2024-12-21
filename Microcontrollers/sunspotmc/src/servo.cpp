#include "servo.h"

// Servo pins
const int X_MIN = 790;
const int X_MAX = 2180;
const int Y_MIN = 780;
const int Y_MAX = 2250;
const int inputmin = -90;
const int inputmax = 90;

const int xaxis = 0;
const int yaxis = 1;

//theta.writeMicroseconds(map(90, -90, 90, 790, 2180));
//beta.writeMicroseconds(map(-90, -90, 90, 780, 2250));
// Function to map a value from one range to another


int pwmout(int angle, int axis) {
    if (axis == xaxis) {
        return map(angle, inputmin, inputmax, X_MIN, X_MAX);
    } else if (axis == yaxis) {
        return map(angle, inputmin, inputmax, Y_MIN, Y_MAX);
    }
    return 0;
}

void moveServo(Servo& servo, int angle, int axis) {
    if (axis == xaxis) {
        servo.writeMicroseconds(pwmout(angle,axis));
    } else if (axis == yaxis) {
        servo.writeMicroseconds(pwmout(angle,axis));
    }
   
}

//connect the control wire into pwm pins and power into 5v and ground into ground
// ensure the controller shares a common ground with the servo(will happen on the board)