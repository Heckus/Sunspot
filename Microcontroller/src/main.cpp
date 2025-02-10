
#include "led.h"
#include "servo.h"
#include "switch.h"
#include "buttons.h"
#include "serial.h"

#define LEDBOARD0_PIN D10
#define LEDNUM0 1
#define LEDBOARD1_PIN D8
#define LEDNUM1 8
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

CapacitiveButton button1(BUTTON1PIN, 100000);
CapacitiveButton button2(BUTTON2PIN, 100000);
CapacitiveButton button3(BUTTON3PIN, 100000);


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
int batteryLevel = 100;

String led0 = "black";

int switchState = 1;
int button1State = 0;
int button2State = 0;
int button3State = 0;

// int button1debouce = 0;
// int button2debouce = 0;
// int button3debouce = 0;
// int buttondebouncereset = 0;

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
    serialComm.waitTillConnected();
    servotheta.attach(SERVOthetaPIN);
    servobeta.attach(SERVObetaPIN);
    
    moveServo(servotheta, 0, xaxis);
    moveServo(servobeta, 0, yaxis);

    setLEDColor(&LED2, 0, "red");
    setLEDColor(&LED2, 1, "red");
    setLEDColor(&LED2, 2, "red");
}



void loop() {
    // Read Pi commands
    String serialinput = serialComm.receive();
    // Check if there is no data
    if(serialinput.length() == 0) {
        nodata = true;
    }else{
    nodata = false;
    if (serialinput == "RESET"){
        ESP.restart();
    }
    
    setTheta(theta + serialComm.extractIntValue(serialinput, 1));
    moveServo(servotheta, theta, xaxis);
    setBeta(beta + serialComm.extractIntValue(serialinput, 2));
    moveServo(servobeta, beta, yaxis);

    led0 = serialComm.extractValue(serialinput, 3);
    setLEDColor(&LED0, 0, led0);
    
    batteryLevel = serialComm.extractIntValue(serialinput, 4);
    setBatteryLevel(&LED1, batteryLevel);
    // }

    // Read switch state
    switchState = threeWaySwitch.getPosition();

    if (button1.isPressed()) {
        button1State ^= 1;   
    }
    if (button2.isPressed()) {
        button2State ^= 1; 
    }
    if (button3.isPressed()) {
        button3State ^= 1;  
    }
    setLEDColor(&LED2,0, button1State ? "green" : "red");
    setLEDColor(&LED2,1, button2State ? "green" : "red");
    setLEDColor(&LED2,2, button3State ? "green" : "red");

    
    
    }
    // // send if there received data
    if (nodata == false){
    serialComm.send(theta, beta, led0, batteryLevel, switchState, button1State, button2State, button3State);
    }
}

  
  
  
    
  