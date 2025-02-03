
#include "led.h"
#include "servo.h"
#include "switch.h"
#include "buttons.h"
#include "serial.h"

#define LEDBOARD0_PIN D10
#define LEDNUM0 1
#define LEDBOARD1_PIN D8
#define LEDNUM1 7
#define LEDBOARD2_PIN D9
#define LEDNUM2 3


#define SERVOthetaPIN D4 // D4 5
#define SERVObetaPIN D5 // D5 6

#define SWITCHPIN A0 // A3 4
#define BUTTON1PIN T2 // Touch 1
#define BUTTON2PIN T3 // Touch 2
#define BUTTON3PIN T4 // Touch 3

/*   OBJECTS  */ 

Adafruit_NeoPixel LED0(LEDNUM0, LEDBOARD0_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel LED1(LEDNUM1, LEDBOARD1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel LED2(LEDNUM2, LEDBOARD2_PIN, NEO_GRB + NEO_KHZ800);

Servo servotheta;
Servo servobeta;

ThreeWaySwitch threeWaySwitch(SWITCHPIN);

CapacitiveButton button1(BUTTON1PIN, 40000);
CapacitiveButton button2(BUTTON2PIN, 40000);
CapacitiveButton button3(BUTTON3PIN, 40000);


SerialComm serialComm(115200);

/*  VARIABLES   */
int theta = 0;
void setTheta(int value) {
    if (value > 90) {
        theta = 90;
    } else if (value < -90) {
        theta = -90;
    } else {
        theta = value;
    }
}
int beta = 0;
void setBeta(int value) {
    if (value > 90) {
        beta = 90;
    } else if (value < -90) {
        beta = -90;
    } else {
        beta = value;
    }
}
int batteryLevel = 8;

String led0 = "red";

int switchState = 1;

int button1State = 0;

int button2State = 0;

int button3State = 0;


/* FLOW CONTROL */

bool nodata = false;

void setup() {
    serialComm.Begin();
    LED0.begin();
    LED1.begin();
    LED2.begin();
    LED0.show(); // Initialize all pixels to 'off'
    LED1.show(); // Initialize all pixels to 'off'
    LED2.show(); // Initialize all pixels to 'off'
    servotheta.attach(SERVOthetaPIN);
    servobeta.attach(SERVObetaPIN);
    moveServo(servotheta, 0, xaxis);
    moveServo(servobeta, 0, yaxis);
    delay(10);
    setLEDColor(&LED2, 0, "red");
    setLEDColor(&LED2, 1, "red");
    setLEDColor(&LED2, 2, "red");
    
}



void loop() {


    setTheta(theta + random(-10, 10));
    moveServo(servotheta, theta, xaxis);
    setBeta(beta + random(-10, 10));
    moveServo(servobeta, beta, yaxis);

    setLEDColor(&LED0, 0, led0);
    led0 = led0 == "red" ? "green" : "red";
    
    batteryLevel = random(0, 100);
    setBatteryLevel(&LED1, batteryLevel);
    delay(500);


}
  
  
  
    
  