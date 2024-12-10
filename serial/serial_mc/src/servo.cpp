#include "servo.h"

#include <Arduino.h>
#include <Servo.h>
// Create servo objects

// Servo pins
const int X_MIN = 23;
const int X_MAX = 157;
const int Y_MIN = 0;
const int Y_MAX = 180;
const int inputmin = -90;
const int inputmax = 90;

const int xaxis = 0;
const int yaxis = 1;

// Function to map a value from one range to another
int move(int value, int axis) { 
    if (axis == 0) {
        return map(value, inputmin, inputmax, X_MIN, X_MAX);
    } else if (axis == 1) {
        return map(value, inputmin, inputmax, Y_MIN, Y_MAX);
    }
    return 0; // Default return value if axis is neither 'x' nor 'y'
}


//connect the control wire into pwm pins and power into 5v and ground into ground
// ensure the controller shares a common ground with the servo(will happen on the board)