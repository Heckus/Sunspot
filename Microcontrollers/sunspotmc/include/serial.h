#pragma once
#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H
#define RXD1 D7  // Hardware UART1 RX pin on Seeed Studio ESP32-S3
#define TXD1 D6  // Hardware UART1 TX pin on Seeed Studio ESP32-S3
#include <Arduino.h>

class SerialComm {
public:
    long baudRate;
    SerialComm(long baudRate);

    void waitTillConnected();
    void send(const String& message);
    void send(int theta, int beta, String led0, int batteryLevel, int SwitchState, int Button1, int Button2, int Button3);
    void debug(const String& message);
    String receive();
    int extractIntValue(String message, int cmd);
    String extractValue(String message, int cmd);

private:
    // Add any private members if needed
};

#endif // SERIAL_COMM_H


// int theta = 0;
// int beta = 0;

// String led0 = "red";
// SerialComm serialComm(9600);

// // Example usage
// void setup() {
//     servotheta.attach(SERVOthetaPIN);
//     servobeta.attach(SERVObetaPIN);
//     servotheta.write(move(0,xaxis));
//     servobeta.write(move(0,yaxis));
//    serialComm.waitTillConnected();
// }

// void loop() {
//     String message = serialComm.receive();
//     if (message.length() == 0) {
//         return;
//     }
//     int cmd = serialComm.extractcmd(message);
//     switch (cmd) {
//         case 2:
//             serialComm.send(theta, beta, led0, 75, 3, 0, 1, 0);
//             break;
//         case 3:
//             theta = 0;
//             beta = 0;
//             led0 = "red";
//             break;
//         case 4:
//             setTheta(theta + serialComm.extractValue(message, cmd).toInt());
//             break;
//         case 5:
//             setBeta(beta + serialComm.extractValue(message, cmd).toInt());
//             break;
//         case 6:
//             // Change LED color
//             led0 = serialComm.extractValue(message, cmd);
//             break;
//         default:
//             // Handle unknown command
//             break;
//     }
//     servotheta.write(move(theta,xaxis));
//     servobeta.write(move(beta,yaxis));
// }
