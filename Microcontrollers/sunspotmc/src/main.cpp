
// #include "led.h"
// #include "servo.h"
// #include "switch.h"
// #include "buttons.h"
// #include "serial.h"

// #define LEDBOARD0 D10 // D10 9
// #define LEDNUM0 1
// #define LEDBOARD1 D9 // D9 8
// #define LEDNUM1 8
// #define LEDBOARD2 D8 // D8 7
// #define LEDNUM2 8


// #define SERVOthetaPIN D4 // D4 5
// #define SERVObetaPIN D5 // D5 6

// #define SWITCHPIN A3 // A3 4
// #define BUTTON1PIN T1 // Touch 1
// #define BUTTON2PIN T2 // Touch 2
// #define BUTTON3PIN T3 // Touch 3


// //LEDBoard statusBoard(LEDBOARD0, LEDNUM0);   // 1 LEDs on pin 7
// //LEDBoard batteryBoard(LEDBOARD1, LEDNUM1);   // 8 LEDs on pin 6
// //LEDBoard buttonBoard(LEDBOARD2,LEDNUM2); // 3 LEDs on pin 5

// // Servo servotheta;
// // Servo servobeta;

// // ThreeWaySwitch threeWaySwitch(SWITCHPIN);

// // CapacitiveButton button1(BUTTON1PIN, 500);
// // CapacitiveButton button2(BUTTON2PIN, 500);
// // CapacitiveButton button3(BUTTON3PIN, 500);


// SerialComm serialComm(115200);

// int theta = 0;
// void setTheta(int value) {
//     if (value > 90) {
//         theta = 90;
//     } else if (value < -90) {
//         theta = -90;
//     } else {
//         theta = value;
//     }
// }
// int beta = 0;
// void setBeta(int value) {
//     if (value > 90) {
//         beta = 90;
//     } else if (value < -90) {
//         beta = -90;
//     } else {
//         beta = value;
//     }
// }
// int batteryLevel = 8;

// String led0 = "white";

// int switchState = 1;

// int button1State = 0;
// int button2State = 0;
// int button3State = 0;


// bool nodata = false;

// void setup() {
//     serialComm.waitTillConnected();
// }



// void loop() {
//     // Read Pi commands
//     String serialinput = serialComm.receive();
//     // Check if there is no data
//     if(serialinput.length() == 0) {
//         nodata = true;
//     }else{
//     // Extract values from serial input
//     theta += serialComm.extractIntValue(serialinput, 1);
//     beta += serialComm.extractIntValue(serialinput, 2);
//     led0 = serialComm.extractValue(serialinput, 3);
//     batteryLevel = serialComm.extractIntValue(serialinput, 4);
//     }

//     // send if there received data
//     if (nodata == false){
//     serialComm.send(theta, beta, led0, batteryLevel, switchState, button1State, button2State, button3State);
//     }


// }
  
  
  
    
  