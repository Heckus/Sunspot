// #include <Arduino.h>
// #include <ESP32Servo.h>

// Servo servo1; //theta
// Servo servo2; //beta

// const int servo1Pin = D4; // Change to your servo1 pin //thet
// const int servo2Pin = D5; // Change to your servo2 pin //beta

// void setup() {
//     Serial.begin(115200);

//     // Attach the servos to the pins
//     servo1.attach(servo1Pin);
//     servo2.attach(servo2Pin);

//     servo1.writeMicroseconds(map(0, -90, 90, 790, 2180));
//     servo2.writeMicroseconds(map(0, -90, 90, 780, 2250));
// }

// void loop() {
//     for (int angle = 500; angle <= 1000; angle += 10) {
        
//         Serial.print("Servo1 map starting value: ");
//         Serial.print(angle);
        
//         delay(1000); // Hold for 1 second
//     }
// }

// //lowertest
    

// // full test
// //  for (int angle = -90; angle <= 90; angle ++) {
// //         servo1.writeMicroseconds(map(angle, -90, 90, 544, 2400));
// //         servo2.writeMicroseconds(map(angle, -90, 90, 544, 2400));
// //         Serial.print("Servo1 Angle: ");
// //         Serial.print(angle);
// //         Serial.print(" Servo2 Angle: ");
// //         Serial.println(angle);
// //         delay(1000); // Hold for 1 second
// //     }