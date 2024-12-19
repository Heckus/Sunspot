#include "serial.h"

SerialComm::SerialComm(long baudRate) {
this->baudRate = baudRate;    
}

void SerialComm::waitTillConnected() {
    Serial.begin(baudRate);
    String bootmessage;
    int cmd = 0;
    while (!Serial) {
        delay(100);
    }
    Serial.println("MC_ONLINE");
    Serial.println("Waiting for Boot");
    while (cmd != 1) {
        bootmessage = receive();
        cmd = extractcmd(bootmessage);
    }
    Serial.println("MC_CONNECTED");
}

void SerialComm::send(const String& message) {
    Serial.println(message);
}

void SerialComm::send(int theta,int beta,String led0,int batteryLevel,int SwitchState,int Button1,int Button2,int Button3) {
    String message1 = "THETA:" + String(theta) + " BETA:" + String(beta) + 
                " LED0:" + led0 + " BAT:" + String(batteryLevel);
    String message2 = " SWITCHSTATE:" + String(SwitchState) + " BUTTONSTATE:" + 
            String(Button1) + String(Button2) + String(Button3);
    String message = message1 + message2;
    Serial.println(message);
}

String SerialComm::receive() {
    String message = "";
    if (Serial.available() > 0) {
        message = Serial.readStringUntil('\n');
    }
    return message;
}



int SerialComm::extractcmd(String message) {
    int cmd = 0;
    if (message.startsWith("THETA:")) {
        cmd = 4;
    } else if (message.startsWith("BETA:")) {
        cmd = 5;
    } else if (message.startsWith("LED0:")) {
        cmd = 6;
    } else if(message.startsWith("STATUS")) {
        cmd = 2;
    } else if(message.startsWith("BOOT")){
        cmd = 1;
    }else if(message.startsWith("RESET")){
        cmd = 3;
    }else{
        cmd = -1;
    }
    return cmd;
}

String  SerialComm::extractValue(String message, int cmd) {
    String value = "";
    if (cmd == 4) {
        value = message.substring(6);
    } else if (cmd == 5) {
        value = message.substring(5);
    } else if (cmd == 6) {
        value = message.substring(5);
    }
    return value;
}




