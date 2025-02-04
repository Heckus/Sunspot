// #include "switch.h"
// #include "buttons.h"




// #define SWITCHPIN A0 // A3 4
// #define BUTTON1PIN T2 // Touch 1
// #define BUTTON2PIN T3 // Touch 2
// #define BUTTON3PIN T4 // Touch 3

// CapacitiveButton button1(BUTTON1PIN, 100000);
// CapacitiveButton button2(BUTTON2PIN, 100000);
// CapacitiveButton button3(BUTTON3PIN, 100000);

// ThreeWaySwitch threeWaySwitch(SWITCHPIN);

// void setup(){
//     Serial.begin(115200);
//     threeWaySwitch.calibrate();
// }

// void loop(){
   
//     threeWaySwitch.printState();
//     Serial.println("buttonsensor1");
//     button1.printSensorValue();
//     Serial.println("buttonsensor2");
//     button2.printSensorValue();
//     Serial.println("buttonsensor3");
//     button3.printSensorValue();

//     Serial.print("button1state = ");
//     Serial.print(button1.isPressed());
//     Serial.print(" button2state = ");
//     Serial.print(button2.isPressed());
//     Serial.print(" button3state = ");
//     Serial.println(button3.isPressed());

    
//     delay(100);
// }