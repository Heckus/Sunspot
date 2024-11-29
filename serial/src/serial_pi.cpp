#include <lgpio.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>

// Serial communication constants
const int BAUD_RATE = 115200;
const uint8_t START_MARKER = 0x7E;
const uint8_t END_MARKER = 0x7F;

// Error checking function
bool validate_checksum(uint32_t message) {
    // Extract individual components
    uint8_t servo1_angle = message & 0x7F;
    uint8_t servo2_angle = (message >> 7) & 0x7F;
    uint8_t param = (message >> 14) & 0x7F;
    uint8_t received_checksum = (message >> 21) & 0x7FF;
    
    // Calculate checksum using XOR
    uint8_t calculated_checksum = servo1_angle ^ servo2_angle ^ param;
    
    return received_checksum == calculated_checksum;
}

// Decode and print message details
void decode_message(uint32_t message) {
    uint8_t servo1_angle = message & 0x7F;
    uint8_t servo2_angle = (message >> 7) & 0x7F;
    uint8_t param = (message >> 14) & 0x7F;

    printf("Servo 1 Angle: %d\n", servo1_angle);
    printf("Servo 2 Angle: %d\n", servo2_angle);
    printf("Parameter: %d\n", param);
}

// Receive message with advanced error checking
uint32_t receive_message(int handle) {
    uint8_t buffer[4];
    uint8_t byte;
    int buffer_index = 0;
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;

    while (1) {
        // Read a byte
        int bytes_read = lgSerialRead(handle, reinterpret_cast<char*>(&byte), 1);
        
        if (bytes_read <= 0) {
            // No data available, small delay to prevent busy waiting
            usleep(10000);  // 10ms delay
            continue;
        }

        // Wait for start marker
        if (buffer_index == 0 && byte != START_MARKER) {
            continue;
        }

        // Collect message bytes
        if (byte == START_MARKER && buffer_index == 0) {
            // Reset for new message
            buffer_index = 0;
            continue;
        }

        // Collect message bytes
        if (buffer_index < 4) {
            buffer[buffer_index++] = byte;
        }

        // Check for end marker and complete message
        if (byte == END_MARKER && buffer_index == 4) {
            // Reconstruct 32-bit message
            uint32_t message = 
                (buffer[0] << 24) | 
                (buffer[1] << 16) | 
                (buffer[2] << 8)  | 
                buffer[3];
            
            // Validate message
            if (validate_checksum(message)) {
                consecutive_errors = 0;
                return message;
            } else {
                consecutive_errors++;
                printf("Checksum error. Consecutive errors: %d\n", consecutive_errors);
                
                // Handle repeated errors
                if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                    fprintf(stderr, "Too many consecutive errors. Resetting communication.\n");
                    // Potential recovery: reset serial connection, send error signal back to sender
                    consecutive_errors = 0;
                }
            }

            // Reset for next message
            buffer_index = 0;
        }
    }
}

int main() {
    const char* device = "/dev/ttyACM1";  // Adjust as needed
    int handle;

    // Open serial connection
    handle = lgSerialOpen(device, BAUD_RATE, 0);
    if (handle < 0) {
        fprintf(stderr, "Could not open serial port %s\n", device);
        return 1;
    }

    // Main receive loop
    try {
        while (1) {
            uint32_t message = receive_message(handle);
            decode_message(message);
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        lgSerialClose(handle);
        return 1;
    }

    lgSerialClose(handle);
    return 0;
}