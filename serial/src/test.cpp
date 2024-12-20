#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>

class SerialCommunicator {
public:
    SerialCommunicator(const char* port, int baudRate) : serialPort(port), baudRate(baudRate), serialHandle(-1) {}

    bool initialize() {
        if ((serialHandle = serialOpen(serialPort, baudRate)) < 0) {
            std::cerr << "Error opening serial port!" << std::endl;
            return false;
        }

        if (wiringPiSetup() == -1) {
            std::cerr << "Error setting up WiringPi!" << std::endl;
            return false;
        }

        std::cout << "Serial communication started. Type 'q' to quit." << std::endl;
        return true;
    }

    void run() {
        while (true) {
            if (serialDataAvail(serialHandle)) {
                char receivedChar = serialGetchar(serialHandle);
                std::cout << "Received: " << receivedChar << std::endl;
            }

            char toSend;
            std::cout << "Enter a character to send: ";
            std::cin >> toSend;

            if (toSend == 'q') break;

            serialPutchar(serialHandle, toSend);
        }
    }

    ~SerialCommunicator() {
        if (serialHandle >= 0) {
            serialClose(serialHandle);
        }
    }

private:
    const char* serialPort;
    int baudRate;
    int serialHandle;
};

int main() {
    SerialCommunicator communicator("/dev/ttyACM0", 115200);

    if (!communicator.initialize()) {
        return 1;
    }

    communicator.run();
    return 0;
}
