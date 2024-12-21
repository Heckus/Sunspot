#include "serial.h"

SerialComm::SerialComm(long baudRate) {
this->baudRate = baudRate;    
}

void SerialComm::waitTillConnected() {
    Serial.begin(baudRate);  // For USB communication
    Serial1.begin(baudRate, SERIAL_8N1, RXD1, TXD1);  // For Pi communication using hardware UART1
    bool waiting = true;
    debug("MC_ONLINE");
    debug("Waiting for Boot");
    while(waiting) {
        if (receive() == "BOOT"){waiting = false;}
        Serial1.println("Waiting for Boot1");
        Serial.println("Waiting for Boot");
        delay(10);
    }
    debug("MC_CONNECTED");
}

void SerialComm::send(const String& message) {
    Serial1.println(message);
}

void SerialComm::send(int theta, int beta, String led0, int batteryLevel, int mode, int input1, int input2, int input3) {
    String message = "MC-THETA:" + String(theta) + "BETA:" + String(beta) + 
                     "LED0:" + led0 + "BAT:" + String(batteryLevel) + 
                     "MODE:" + String(mode) + "INPUT:" + String(input1) + 
                     String(input2) + String(input3);
    Serial1.println(message);
}

String SerialComm::receive() {
    String message = "";
    if (Serial1.available() > 0) {
        message = Serial1.readStringUntil('\n');
    }
    return message;
}

void SerialComm::debug(const String& message) {
    Serial.println(message);
}



String SerialComm::extractValue(String message, int cmd) {
    String value = "";
    int startIndex = 0;
    int endIndex = 0;

    switch (cmd) {
        case 1: // THETA
            startIndex = message.indexOf("THETA:") + 4;
            endIndex = message.indexOf("BETA:");
            value = message.substring(startIndex, endIndex);
            break;
        case 2: // BETA
            startIndex = message.indexOf("BETA:") + 4;
            endIndex = message.indexOf("LED0:");
            value = message.substring(startIndex, endIndex);
            break;
        case 3: // LED0
            startIndex = message.indexOf("LED0:") + 5;
            endIndex = message.indexOf("BAT:");
            value = message.substring(startIndex, endIndex);
            break;
        case 4: // BAT
            startIndex = message.indexOf("BAT:") + 4;
            value = message.substring(startIndex);
            break;
        default:
            break;
    }

    return value;
}

int SerialComm::extractIntValue(String message, int cmd) {
    return extractValue(message, cmd).toInt();
}




