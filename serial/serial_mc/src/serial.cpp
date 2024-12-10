#include <Arduino.h>

class SerialComm {
public:
    long baudRate;
    SerialComm(long baudRate) {
    this->baudRate = baudRate;    
    }

    void waitTillConnected() {
        Serial.begin(baudRate);
        while (!Serial) {
            delay(100);
        }
        Serial.println("MC_CONNECTED");
    }

    void send(const String& message) {
        Serial.println(message);
    }

    void send(int theta,int beta,String led0,int batteryLevel,int SwitchState,int Button1,int Button2,int Button3) {
        String message1 = "THETA:" + String(theta) + " BETA:" + String(beta) + 
                 " LED0:" + led0 + " BAT:" + String(batteryLevel);
        String message2 = " STATE:" + String(SwitchState, BIN).substring(0, 3) + 
             String(Button1) + String(Button2) + String(Button3);
        String message = message1 + message2;
        Serial.println(message);
    }

    String receive() {
        String message = "";
        if (Serial.available() > 0) {
            message = Serial.readStringUntil('\n');
        }
        return message;
    }


    int extractcmd(String message) {
        int cmd = 0;
        if (message.startsWith("C_THETA:")) {
            cmd = 1;
        } else if (message.startsWith("C_BETA:")) {
            cmd = 2;
        } else if (message.startsWith("LED0:")) {
            cmd = 3;
        }else{
            cmd = -1;
        }
        return cmd;
    }

    String extractValue(String message, int cmd) {
        String value = "";
        if (cmd == 1) {
            value = message.substring(8);
        } else if (cmd == 2) {
            value = message.substring(7);
        } else if (cmd == 3) {
            value = message.substring(5);
        }
        return value;
    }

    

private:
    // Add any private members if needed
};

int theta = 0;
int beta = 0;
String led0 = "red";
SerialComm serialComm(9600);
// Example usage
void setup() {
   serialComm.waitTillConnected();
   
}

void loop() {
    String message = serialComm.receive();
    if (message.length() == 0) {
        return;
    }
    int cmd = serialComm.extractcmd(message);
    if (cmd == 1) {
        theta += serialComm.extractValue(message, cmd).toInt();
    } else if (cmd == 2) {
        beta += serialComm.extractValue(message, cmd).toInt();
    } else if (cmd == 3) {
        // Change LED color
        led0 = serialComm.extractValue(message, cmd);
    } else if (message == "OK") {
        serialComm.send(theta, beta, led0, 75, 3, 0, 1, 0);
    }
}