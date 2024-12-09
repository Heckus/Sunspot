
#include "led.h"
#include "servo.h"

#define LEDBOARD0 7
#define LEDNUM0 1
#define LEDBOARD1 6
#define LEDNUM1 8
#define LEDBOARD2 5
#define LEDNUM2 3


#define SERVOthetaPIN 9
#define SERVObetaPIN 10


LEDBoard statusBoard(LEDBOARD0, LEDNUM0);   // 1 LEDs on pin 7
LEDBoard batteryBoard(LEDBOARD1, LEDNUM1);   // 8 LEDs on pin 6
LEDBoard buttonBoard(LEDBOARD2,LEDNUM2); // 3 LEDs on pin 5

Servo servotheta;
Servo servobeta;


void setup() {
    Serial.begin(9600);
    FastLED.setBrightness(50);
    servotheta.attach(SERVOthetaPIN);
    servobeta.attach(SERVObetaPIN);
    servotheta.write(move(0,xaxis));
    servobeta.write(move(0,yaxis));
    statusBoard.setLED(0, 255, 255, 255);    // Set camera LED to white
    // while (!Serial) {
    //     // Wait for serial connection
    // }
    // Serial.println("Arduino ready.");
}

void loop() {
    // Check if data is available from Raspberry Pi
    // if (Serial.available() > 0) {
    //     String message = Serial.readStringUntil('\n'); // Read until newline
    //     Serial.print("I got this message: ");
    //     Serial.println(message); // Send acknowledgment
    // }
  
    batteryBoard.setBatteryLevel(100);
    delay(1000);
    batteryBoard.setBatteryLevel(50);
    delay(1000);
    batteryBoard.setBatteryLevel(10);
    delay(1000);
    batteryBoard.setBatteryLevel(0);
    delay(1000);
  
  for (int i = -90; i <= 90; i++) {
        servotheta.write(move(i,xaxis));
        servobeta.write(move(-i,yaxis));
        delay(20);
    }
    for (int i = 90; i >= -90; i--) {
        servotheta.write(move(i,xaxis));
        servobeta.write(move(-i,yaxis));
        delay(20);
    }
    delay(1000);
    servotheta.write(move(0,xaxis));
    servobeta.write(move(0,yaxis));
    delay(1000);
  // Other loop logic can go here
  // This ensures millis() continues to be called

}

