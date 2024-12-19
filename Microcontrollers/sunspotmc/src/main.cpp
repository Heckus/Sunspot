
#include "led.h"
#include "servo.h"
#include "switch.h"
#include "buttons.h"
#include "serial.h"

#define LEDBOARD0 18
#define LEDNUM0 1
#define LEDBOARD1 19
#define LEDNUM1 8
#define LEDBOARD2 21
#define LEDNUM2 3


#define SERVOthetaPIN 9
#define SERVObetaPIN 10

#define SWITCHPIN A0
#define BUTTON1PIN 3
#define BUTTON2PIN 13
#define BUTTON3PIN 11


LEDBoard statusBoard(LEDBOARD0, LEDNUM0);   // 1 LEDs on pin 7
LEDBoard batteryBoard(LEDBOARD1, LEDNUM1);   // 8 LEDs on pin 6
LEDBoard buttonBoard(LEDBOARD2,LEDNUM2); // 3 LEDs on pin 5

Servo servotheta;
Servo servobeta;

ThreeWaySwitch threeWaySwitch(SWITCHPIN);

CapacitiveButton button1(BUTTON1PIN, 10000);
CapacitiveButton button2(BUTTON2PIN, 10000);
CapacitiveButton button3(BUTTON3PIN, 10000);


SerialComm serialComm(19200);

int theta =0;
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

String led0 = "white";

int switchState = 1;

int button1State = 0;
int button2State = 0;
int button3State = 0;
int button4State = 0;


void setup() {
    statusBoard.clear();
    batteryBoard.clear();
    buttonBoard.clear();
    serialComm.waitTillConnected();
    FastLED.setBrightness(75);
    servotheta.attach(SERVOthetaPIN);
    servobeta.attach(SERVObetaPIN);
    servotheta.write(move(0,xaxis));
    servobeta.write(move(0,yaxis));
    statusBoard.setLED(0,LEDBoard::STCRGB(led0));    // Set camera LED to white
    batteryBoard.setBatteryLevel(100);
    buttonBoard.setLED(0,CRGB::Red);
    buttonBoard.setLED(1,CRGB::Red);
    buttonBoard.setLED(2,CRGB::Red);
    
}

int skip = 0;
void loop() {
    String message = serialComm.receive();
    if (message.length() == 0) {
        return;
    }
    int cmd = serialComm.extractcmd(message);
    switch (cmd) {
        case 2: //Status
            skip = 1;
            break;
        case 3: //RESET
            theta = 0;
            beta = 0;
            statusBoard.clear();
            batteryBoard.clear();
            buttonBoard.clear();
            switchState = 1;
            button1State = 0;
            button2State = 0;
            button3State = 0;
            button4State = 0;
            asm volatile ("  jmp 0");
            break;
        case 4:
            setTheta(theta + serialComm.extractValue(message, cmd).toInt());
            servotheta.write(move(theta,xaxis));

            
            break;
        case 5:
            setBeta(beta + serialComm.extractValue(message, cmd).toInt());
            servobeta.write(move(beta,xaxis));
            break;
        case 6:
            led0 = serialComm.extractValue(message, cmd);
            statusBoard.setLED(0,LEDBoard::STCRGB(led0));
            break;
        default:
            break;
    }
    
    if (button1.isPressed()) {
        button1State ^= 1;
        buttonBoard.setLED(0, button1State ? CRGB::Green : CRGB::Red);
    }
    // if (button2.isPressed()) {
    //     button2State ^= 1;
    //     buttonBoard.setLED(1, button2State ? CRGB::Green : CRGB::Red);
    // }
    // if (button3.isPressed()) {
    //     button3State ^= 1;
    //     buttonBoard.setLED(2, button3State ? CRGB::Green : CRGB::Red);
    // }
    switchState = threeWaySwitch.getPosition();
    batteryBoard.setBatteryLevel(batteryLevel);

    if (skip == 1) {
        serialComm.send(theta, beta, led0, batteryLevel, switchState, button1State, button2State, button3State);
        skip = 0;
    }
    delay(100);
}
  
  
  
    
  