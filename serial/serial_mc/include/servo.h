#pragma once
#ifndef SERVO_H
#define SERVO_H
#include <Arduino.h>
#include <Servo.h>

extern const int X_MIN;
extern const int X_MAX;
extern const int Y_MIN;
extern const int Y_MAX;
extern const int inputmin;
extern const int inputmax;

extern const int xaxis;
extern const int yaxis;

// Function to map a value from one range to another
int move(int value, int axis);
#endif // SERVO_H


//Example use 

// Servo servoX;
// Servo servoY;
// void setup() {
//   // Attach servos to their pins
//   servoX.attach(9);
//   servoY.attach(10);
  
//   // Initialize serial communication
//   Serial.begin(9600);
  
//   // Center both servos on startup
//   servoX.write(move(0,xaxis));
//   servoY.write(move(0,yaxis));
// }

// void loop() {
//     for (int i = -90; i <= 90; i++) {
//         servoX.write(move(i,xaxis));
//         servoY.write(move(-i,yaxis));
//         delay(20);
//     }
//     for (int i = 90; i >= -90; i--) {
//         servoX.write(move(i,xaxis));
//         servoY.write(move(-i,yaxis));
//         delay(20);
//     }
//     delay(1000);
//     servoX.write(move(0,xaxis));
//     servoY.write(move(0,yaxis));
//     delay(1000);
// }
