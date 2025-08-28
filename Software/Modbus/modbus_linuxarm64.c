#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <sys/select.h>
#include <ctype.h>
#include <termios.h>

#define HISTORY_SIZE 50
#define MAX_CMD_LEN 256

// Command history handling
static char cmd_history[HISTORY_SIZE][MAX_CMD_LEN];
static int history_count = 0;
static int history_pos = 0;
static struct termios orig_termios;

// Save original terminal settings and setup raw mode
static void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// Restore original terminal settings
static void disable_raw_mode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

// Add command to history
static void add_to_history(const char* cmd) {
    if (strlen(cmd) == 0) return;
    
    // Don't add if it's the same as the last command
    if (history_count > 0 && strcmp(cmd_history[history_count - 1], cmd) == 0) {
        return;
    }
    
    if (history_count < HISTORY_SIZE) {
        strncpy(cmd_history[history_count], cmd, MAX_CMD_LEN - 1);
        cmd_history[history_count][MAX_CMD_LEN - 1] = '\0';
        history_count++;
    } else {
        // Shift history down
        for (int i = 0; i < HISTORY_SIZE - 1; i++) {
            strcpy(cmd_history[i], cmd_history[i + 1]);
        }
        strncpy(cmd_history[HISTORY_SIZE - 1], cmd, MAX_CMD_LEN - 1);
        cmd_history[HISTORY_SIZE - 1][MAX_CMD_LEN - 1] = '\0';
    }
    history_pos = history_count;
}

// Timer implementation for Linux
static uint32_t get_timer_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec * 1000) + (ts.tv_nsec / 1000000));
}

#define TIMER_SET() get_timer_ms()
#define TIMER_ELAPSED_MS(t, ms) ((get_timer_ms() - (t)) > (ms))

// Modbus definitions
#define UART_BUFFER_SIZE 128
#define INPUTS_SIZE 6
#define OUTPUTS_SIZE 6
#define EXTRA_ACTION_PER_INPUT 4

// Modbus function codes
#define READ_COILS              0x01 
#define READ_DISC_INPUTS        0x02
#define READ_HOLD_REGS          0x03
#define READ_INPUT_REGS         0x04
#define WRITE_SINGLE_COIL       0x05
#define WRITE_SINGLE_REG        0x06
#define WRITE_MULTI_COILS       0x0F
#define WRITE_MULTI_REGS        0x10

// Error codes
#define HANDLED_OK              0x00
#define ID_MISMATCH             0x01
#define INVALID_DATA_LENGHT     0x02
#define INVALID_COIL_VALUE      0x03
#define INVALID_FUNCTION        0x10
#define NOT_IMPLEMENTED         0xFF

// Action types
typedef enum {
    nop,
    toggle,
    on,
    off,
    offdelayon,
    ondelayoff
} Action;

// Modbus message structure
typedef struct {
    uint8_t slave_address;
    uint8_t function_code;
    uint8_t data[100];
    uint8_t data_length;
} ModbusMessage;

// EventAction structure
typedef struct {
    Action action;
    Action delayAction;
    uint32_t delay;
    uint8_t pwm;
    uint8_t id;
    uint8_t output;
    uint8_t send;
    uint8_t extraEventId;
} EventAction;

// InputAction structure
typedef struct {
    EventAction singlePress;
    EventAction doublePress;
    EventAction longPress;
    EventAction switchOn;
    EventAction switchOff;
} InputAction;

// Global variables
static int uart_fd = -1;
static uint8_t MODBUS_ID = 1;
static uint8_t uart_rxBuffer[UART_BUFFER_SIZE] = {0};
static uint8_t uart_txBuffer[UART_BUFFER_SIZE] = {0};
static uint8_t new_rxdata = 0;
static uint16_t rxDataLen = 0;

// Modbus arrays
static uint8_t mbCoilsArray[1] = {0x00};
static uint8_t mbInputsArray[1] = {0x00};
static uint16_t mbHRegArray[50] = {0};
static uint16_t mbIRegArray[1] = {0};

static uint8_t* modbusCoils = mbCoilsArray;
static uint8_t* modbusInputs = mbInputsArray;
static uint16_t* modbusHReg = mbHRegArray;
static uint16_t* modbusIReg = mbIRegArray;

static uint8_t waiting4response = 0;
static uint32_t resend_timer = 0;
static uint8_t resend_count = 0;
static ModbusMessage send_message;
static bool interactive_mode = false;
static bool command_pending = false;

// CRC calculation for Modbus RTU
static uint16_t calculate_crc(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 8; j > 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return (uint16_t)(crc << 8) | (uint16_t)(crc >> 8);
}

// Decode Modbus RTU message
static uint8_t decode_modbus_rtu(uint8_t* message, size_t length, ModbusMessage* decoded) {
    // Check minimum length (addr + func + crc = 4 bytes min)
    if (length < 4) {
        printf("Error: Message too short (%d bytes)\n", (int)length);
        return 255;
    }
    
    if (message[0] == 0) {
        printf("Error: Invalid slave address (0)\n");
        return 255;
    }
    
    // In master mode, we should accept responses from the slave we queried
    if (waiting4response && message[0] != send_message.slave_address) {
        printf("Error: Unexpected slave address %d, expecting %d\n", message[0], send_message.slave_address);
        return message[0];
    }
    // In slave mode, only accept messages for our ID or broadcast
    else if (!waiting4response && message[0] != MODBUS_ID && message[0] != 250) {
        printf("Error: Message not for us (ID %d)\n", message[0]);
        return message[0];
    }

    // Check CRC
    uint16_t crc_calc = calculate_crc(message, length - 2);
    uint16_t crc_recv = (uint16_t)(message[length - 2] << 8) | (uint16_t)(message[length - 1]);
    if (crc_recv != crc_calc) {
        printf("Error: CRC mismatch (received 0x%04X, calculated 0x%04X)\n", crc_recv, crc_calc);
        return 1;
    }

    // Parse message
    decoded->slave_address = message[0];
    decoded->function_code = message[1];
    
    for (uint8_t i = 2; i < (length - 2); i++) {
        decoded->data[i - 2] = message[i];
        decoded->data_length = (uint8_t)(i - 1);
    }

    return 0;
}

// Encode Modbus RTU message
static uint8_t encode_modbus_rtu(uint8_t* encoded, size_t* length, ModbusMessage* message) {
    encoded[0] = message->slave_address;
    encoded[1] = message->function_code;
    *length = 4; // address + function_code + crc

    for (uint8_t i = 2; i < (message->data_length + 2); i++) {
        encoded[i] = message->data[i - 2];
        *length = *length + 1;
    }

    // Calculate CRC
    uint16_t crc_calc = calculate_crc(encoded, *length - 2);
    encoded[*length - 1] = crc_calc;
    encoded[*length - 2] = crc_calc >> 8;

    return 0;
}

// Handle Modbus messages
static uint8_t modbusm_handle(ModbusMessage* message) {
    switch (message->function_code) {
    case READ_COILS: {
        uint16_t first_coil = (message->data[0] << 8) | message->data[1];
        uint16_t amount_coils = (message->data[2] << 8) | message->data[3];
        
        if (message->data_length != 4) return INVALID_DATA_LENGHT;

        message->slave_address = MODBUS_ID;
        message->function_code = READ_COILS;
        message->data_length = amount_coils % 8 ? 2 + (amount_coils / 8) : 1 + (amount_coils / 8);
        message->data[0] = message->data_length - 1;
        
        for (int i = 1; i < message->data_length; i++) {
            uint8_t currentB = *(modbusCoils + ((first_coil / 8) + (i - 1)));
            uint8_t nextB = *(modbusCoils + ((first_coil / 8) + (i)));
            *(message->data + i) = (nextB << (8 - (first_coil % 8)) | currentB >> (first_coil % 8));
        }
        break;
    }
    
    case READ_HOLD_REGS: {
        uint16_t first_hreg = (message->data[0] << 8) | message->data[1];
        uint16_t amount_hregs = (message->data[2] << 8) | message->data[3];
        
        if (message->data_length != 4) return INVALID_DATA_LENGHT;

        message->slave_address = MODBUS_ID;
        message->function_code = READ_HOLD_REGS;
        message->data_length = 1 + (amount_hregs * 2);
        message->data[0] = message->data_length - 1;
        
        for (int i = 1; i < message->data_length; i++) {
            *(message->data + (i * 2)) = *(modbusHReg + i - 1 + first_hreg) >> 8;
            *(message->data + (i * 2) - 1) = *(modbusHReg + i - 1 + first_hreg);
        }
        break;
    }

    case WRITE_SINGLE_COIL: {
        uint16_t coil_address = (message->data[0] << 8) | message->data[1];
        uint16_t value = (message->data[2] << 8) | message->data[3];
        
        if (message->data_length != 4) return INVALID_DATA_LENGHT;

        if (value == 0xFF00) {
            modbusCoils[coil_address / 8] |= (1 << (coil_address % 8));
        } else if (value == 0x0000) {
            modbusCoils[coil_address / 8] &= ~(1 << (coil_address % 8));
        } else if (value == 0x5555) {  // Toggle operation
            modbusCoils[coil_address / 8] ^= (1 << (coil_address % 8));
        } else {
            return INVALID_COIL_VALUE;
        }
        break;
    }

    case WRITE_SINGLE_REG: {
        uint16_t reg_address = (message->data[0] << 8) | message->data[1];
        
        if (message->data_length != 4) return INVALID_DATA_LENGHT;
        
        modbusHReg[reg_address] = (message->data[2] << 8) | message->data[3];
        break;
    }

    case WRITE_MULTI_REGS: {
        uint16_t start_address = (message->data[0] << 8) | message->data[1];
        uint16_t num_registers = (message->data[2] << 8) | message->data[3];
        uint8_t byte_count = message->data[4];
        
        if (message->data_length != 5 + byte_count || byte_count != num_registers * 2) {
            return INVALID_DATA_LENGHT;
        }
        
        for (uint8_t i = 0; i < num_registers; i++) {
            uint16_t value = (message->data[5 + i * 2] << 8) | message->data[6 + i * 2];
            mbHRegArray[start_address + i] = value;
        }
        
        message->data_length = 4;
        message->data[0] = (start_address >> 8) & 0xFF;
        message->data[1] = start_address & 0xFF;
        message->data[2] = (num_registers >> 8) & 0xFF;
        message->data[3] = num_registers & 0xFF;
        break;
    }

    default:
        return NOT_IMPLEMENTED;
    }

    return HANDLED_OK;
}

// Initialize UART
static int init_uart(const char* device) {
    struct termios tty;
    
    uart_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd < 0) {
        printf("Error opening %s: %s\n", device, strerror(errno));
        return -1;
    }

    // Configure UART
    if (tcgetattr(uart_fd, &tty) != 0) {
        printf("Error getting UART attributes: %s\n", strerror(errno));
        close(uart_fd);
        return -1;
    }

    // Set baud rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1
    tty.c_cflag &= ~PARENB;     // No parity
    tty.c_cflag &= ~CSTOPB;     // One stop bit
    tty.c_cflag &= ~CSIZE;      // Clear size bits
    tty.c_cflag |= CS8;         // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;    // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~ICANON;     // Disable canonical mode
    tty.c_lflag &= ~ECHO;       // Disable echo
    tty.c_lflag &= ~ECHOE;      // Disable erasure
    tty.c_lflag &= ~ECHONL;     // Disable new-line echo
    tty.c_lflag &= ~ISIG;       // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set timeout
    tty.c_cc[VTIME] = 1;    // Wait for up to 0.1s (1 decisecond)
    tty.c_cc[VMIN] = 0;     // No minimum number of characters

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        printf("Error setting UART attributes: %s\n", strerror(errno));
        close(uart_fd);
        return -1;
    }

    // Flush any existing data
    tcflush(uart_fd, TCIOFLUSH);

    printf("UART initialized successfully on %s\n", device);
    return 0;
}

// Read from UART
static int uart_read(void) {
    int bytes_read = read(uart_fd, uart_rxBuffer + rxDataLen, UART_BUFFER_SIZE - rxDataLen - 1);
    if (bytes_read > 0) {
        rxDataLen += bytes_read;
        new_rxdata = 1;
        return bytes_read;
    }
    return 0;
}

// Write to UART
static int uart_write(const uint8_t* data, size_t length) {
    return write(uart_fd, data, length);
}

// Receive and handle Modbus messages
static void modbus_receive(void) {
    ModbusMessage received_message;
    size_t length;

    memset(uart_txBuffer, 0, UART_BUFFER_SIZE);
    
    if (decode_modbus_rtu(uart_rxBuffer, rxDataLen, &received_message) == 0) {
        printf("Received Modbus message from slave %d, function %d\n", 
               received_message.slave_address, received_message.function_code);
        
        if (modbusm_handle(&received_message) != 1) {
            if (received_message.slave_address != 250) {
                length = 0;
                encode_modbus_rtu(uart_txBuffer, &length, &received_message);
                
                printf("Sending response: ");
                for (int i = 0; i < length; i++) {
                    printf("%02X ", uart_txBuffer[i]);
                }
                printf("\n");
                
                uart_write(uart_txBuffer, length);
            }
        }
    } else {
        printf("Invalid Modbus RTU message received\n");
    }
    
    // Reset for new message
    new_rxdata = 0;
    memset(uart_rxBuffer, 0, UART_BUFFER_SIZE);
    rxDataLen = 0;
}

// Forward declarations
static void modbus_send_message(uint8_t slave_id, uint8_t function, uint16_t address, uint16_t value);
static void process_command(char* input);

// Print help menu
static void print_help(void) {
    printf("\n=== Modbus RTU Command Interface ===\n");
    printf("Commands:\n");
    printf("  rc <slave> <addr> <count>     - Read Coils\n");
    printf("  ri <slave> <addr> <count>     - Read Discrete Inputs\n");
    printf("  rh <slave> <addr> <count>     - Read Holding Registers\n");
    printf("  rr <slave> <addr> <count>     - Read Input Registers\n");
    printf("  wc <slave> <addr> <value>     - Write Single Coil (0=OFF, 1=ON, 2=TOGGLE)\n");
    printf("  wr <slave> <addr> <value>     - Write Single Register\n");
    printf("  wmc <slave> <addr> <values>   - Write Multiple Coils (hex string)\n");
    printf("  wmr <slave> <addr> <values>   - Write Multiple Registers (hex values)\n");
    printf("  status                        - Show current coil/register status\n");
    printf("  setid <id>                    - Set local Modbus ID\n");
    printf("  help                          - Show this help\n");
    printf("  quit                          - Exit program\n");
    printf("\nExamples:\n");
    printf("  rc 2 0 6          - Read 6 coils starting from address 0 on slave 2\n");
    printf("  wc 2 0 1          - Turn ON coil 0 on slave 2\n");
    printf("  wc 2 0 2          - Toggle coil 0 on slave 2\n");
    printf("  rh 3 0 10         - Read 10 holding registers from slave 3\n");
    printf("  wr 2 5 1234       - Write value 1234 to register 5 on slave 2\n");
    printf("=====================================\n\n");
}

// Parse and execute user commands
static void process_command(char* input) {
    char cmd[32];
    char *token, *saveptr;
    int args[10];
    int arg_count = 0;
    
    // Remove newline
    input[strcspn(input, "\n")] = 0;
    
    // Skip empty commands
    if (strlen(input) == 0) return;
    
    // Parse command
    token = strtok_r(input, " ", &saveptr);
    if (!token) return;
    
    strncpy(cmd, token, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = 0;
    
    // Convert to lowercase
    for (int i = 0; cmd[i]; i++) {
        cmd[i] = tolower(cmd[i]);
    }
    
    // Parse arguments
    while ((token = strtok_r(NULL, " ", &saveptr)) && arg_count < 10) {
        if (strncmp(token, "0x", 2) == 0) {
            args[arg_count] = (int)strtol(token, NULL, 16);
        } else {
            args[arg_count] = atoi(token);
        }
        arg_count++;
    }
    
    // Process commands
    if (strcmp(cmd, "help") == 0) {
        print_help();
    }
    else if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        printf("Exiting...\n");
        exit(0);
    }
    else if (strcmp(cmd, "status") == 0) {
        printf("\n=== Local Status ===\n");
        printf("Modbus ID: %d\n", MODBUS_ID);
        printf("Coils (8 bits): 0x%02X\n", modbusCoils[0]);
        printf("Holding Registers (first 10):\n");
        for (int i = 0; i < 10; i++) {
            printf("  Reg[%d] = %d (0x%04X)\n", i, modbusHReg[i], modbusHReg[i]);
        }
        printf("==================\n\n");
    }
    else if (strcmp(cmd, "setid") == 0) {
        if (arg_count >= 1 && args[0] >= 1 && args[0] <= 247) {
            MODBUS_ID = args[0];
            printf("Modbus ID set to %d\n", MODBUS_ID);
        } else {
            printf("Error: Invalid ID (must be 1-247)\n");
        }
    }
    else if (strcmp(cmd, "rc") == 0) { // Read Coils
        if (arg_count >= 3) {
            printf("Reading %d coils from address %d on slave %d\n", args[2], args[1], args[0]);
            modbus_send_message(args[0], READ_COILS, args[1], args[2]);
            command_pending = true;
        } else {
            printf("Error: Usage: rc <slave> <address> <count>\n");
        }
    }
    else if (strcmp(cmd, "ri") == 0) { // Read Discrete Inputs
        if (arg_count >= 3) {
            printf("Reading %d discrete inputs from address %d on slave %d\n", args[2], args[1], args[0]);
            modbus_send_message(args[0], READ_DISC_INPUTS, args[1], args[2]);
            command_pending = true;
        } else {
            printf("Error: Usage: ri <slave> <address> <count>\n");
        }
    }
    else if (strcmp(cmd, "rh") == 0) { // Read Holding Registers
        if (arg_count >= 3) {
            printf("Reading %d holding registers from address %d on slave %d\n", args[2], args[1], args[0]);
            modbus_send_message(args[0], READ_HOLD_REGS, args[1], args[2]);
            command_pending = true;
        } else {
            printf("Error: Usage: rh <slave> <address> <count>\n");
        }
    }
    else if (strcmp(cmd, "rr") == 0) { // Read Input Registers
        if (arg_count >= 3) {
            printf("Reading %d input registers from address %d on slave %d\n", args[2], args[1], args[0]);
            modbus_send_message(args[0], READ_INPUT_REGS, args[1], args[2]);
            command_pending = true;
        } else {
            printf("Error: Usage: rr <slave> <address> <count>\n");
        }
    }
    else if (strcmp(cmd, "wc") == 0) { // Write Single Coil
        if (arg_count >= 3) {
            uint16_t value;
            if (args[2] == 0) {
                value = 0x0000; // OFF
                printf("Setting coil %d OFF on slave %d\n", args[1], args[0]);
            } else if (args[2] == 1) {
                value = 0xFF00; // ON
                printf("Setting coil %d ON on slave %d\n", args[1], args[0]);
            } else if (args[2] == 2) {
                value = 0x5555; // TOGGLE
                printf("Toggling coil %d on slave %d\n", args[1], args[0]);
            } else {
                printf("Error: Coil value must be 0 (OFF), 1 (ON), or 2 (TOGGLE)\n");
                return;
            }
            modbus_send_message(args[0], WRITE_SINGLE_COIL, args[1], value);
            command_pending = true;
        } else {
            printf("Error: Usage: wc <slave> <address> <value>\n");
        }
    }
    else if (strcmp(cmd, "wr") == 0) { // Write Single Register
        if (arg_count >= 3) {
            printf("Writing value %d to register %d on slave %d\n", args[2], args[1], args[0]);
            modbus_send_message(args[0], WRITE_SINGLE_REG, args[1], args[2]);
            command_pending = true;
        } else {
            printf("Error: Usage: wr <slave> <address> <value>\n");
        }
    }
    else if (strcmp(cmd, "wmc") == 0) { // Write Multiple Coils
        if (arg_count >= 3) {
            ModbusMessage msg;
            msg.slave_address = args[0];
            msg.function_code = WRITE_MULTI_COILS;
            
            uint16_t start_addr = args[1];
            uint16_t num_coils = strlen(saveptr) / 2; // Each byte in hex string represents 8 coils
            uint8_t byte_count = (num_coils + 7) / 8; // Round up to nearest byte
            
            msg.data[0] = (start_addr >> 8) & 0xFF;
            msg.data[1] = start_addr & 0xFF;
            msg.data[2] = (num_coils >> 8) & 0xFF;
            msg.data[3] = num_coils & 0xFF;
            msg.data[4] = byte_count;
            
            // Convert hex string to bytes
            char *hex_str = saveptr;
            for (int i = 0; i < byte_count && hex_str[i*2] && hex_str[i*2+1]; i++) {
                char byte_str[3] = {hex_str[i*2], hex_str[i*2+1], 0};
                msg.data[5 + i] = strtol(byte_str, NULL, 16);
            }
            
            msg.data_length = 5 + byte_count;
            
            size_t length;
            encode_modbus_rtu(uart_txBuffer, &length, &msg);
            
            printf("Writing %d coils starting at address %d on slave %d with values: %s\n", 
                   num_coils, start_addr, args[0], saveptr);
            
            uart_write(uart_txBuffer, length);
            waiting4response = 1;
            resend_timer = TIMER_SET();
            command_pending = true;
        } else {
            printf("Error: Usage: wmc <slave> <start_addr> <hex_string>\n");
            printf("Example: wmc 1 0 FF00 - Write 16 coils, first 8 ON, next 8 OFF\n");
        }
    }
    else if (strcmp(cmd, "wmr") == 0) { // Write Multiple Registers
        if (arg_count >= 3) {
            ModbusMessage msg;
            msg.slave_address = args[0];
            msg.function_code = WRITE_MULTI_REGS;
            
            uint16_t start_addr = args[1];
            uint16_t num_regs = arg_count - 2;
            uint8_t byte_count = num_regs * 2;
            
            msg.data[0] = (start_addr >> 8) & 0xFF;
            msg.data[1] = start_addr & 0xFF;
            msg.data[2] = (num_regs >> 8) & 0xFF;
            msg.data[3] = num_regs & 0xFF;
            msg.data[4] = byte_count;
            
            for (int i = 0; i < num_regs; i++) {
                uint16_t value = args[i + 2];
                msg.data[5 + i * 2] = (value >> 8) & 0xFF;
                msg.data[6 + i * 2] = value & 0xFF;
            }
            
            msg.data_length = 5 + byte_count;
            
            size_t length;
            encode_modbus_rtu(uart_txBuffer, &length, &msg);
            
            printf("Writing %d registers starting at address %d on slave %d: ", num_regs, start_addr, args[0]);
            for (int i = 0; i < num_regs; i++) {
                printf("%d ", args[i + 2]);
            }
            printf("\n");
            
            uart_write(uart_txBuffer, length);
            waiting4response = 1;
            resend_timer = TIMER_SET();
            command_pending = true;
        } else {
            printf("Error: Usage: wmr <slave> <start_addr> <value1> [value2] ...\n");
        }
    }
    else {
        printf("Unknown command: %s\n", cmd);
        printf("Type 'help' for available commands\n");
    }
}

// Send a Modbus message
static void modbus_send_message(uint8_t slave_id, uint8_t function, uint16_t address, uint16_t value) {
    ModbusMessage msg;
    msg.slave_address = slave_id;
    msg.function_code = function;
    
    // Prepare data based on function code
    switch (function) {
        case READ_COILS:
        case READ_DISC_INPUTS:
        case READ_HOLD_REGS:
        case READ_INPUT_REGS:
            msg.data[0] = (address >> 8) & 0xFF;
            msg.data[1] = address & 0xFF;
            msg.data[2] = (value >> 8) & 0xFF;  // value represents count for read operations
            msg.data[3] = value & 0xFF;
            msg.data_length = 4;
            break;
            
        case WRITE_SINGLE_COIL:
        case WRITE_SINGLE_REG:
            msg.data[0] = (address >> 8) & 0xFF;
            msg.data[1] = address & 0xFF;
            msg.data[2] = (value >> 8) & 0xFF;
            msg.data[3] = value & 0xFF;
            msg.data_length = 4;
            break;
            
        default:
            printf("Unsupported function code: %d\n", function);
            return;
    }
    
    size_t length;
    encode_modbus_rtu(uart_txBuffer, &length, &msg);
    
    // Save the message we're sending so we can verify the response
    memcpy(&send_message, &msg, sizeof(ModbusMessage));
    
    uart_write(uart_txBuffer, length);
    waiting4response = 1;
    resend_timer = TIMER_SET();
}


// Handle response with detailed output
static void modbus_response(void) {
    if (new_rxdata) {
        ModbusMessage received_message;
        
        if (decode_modbus_rtu(uart_rxBuffer, rxDataLen, &received_message) == 0) {
            printf("✓ Response from slave %d: ", received_message.slave_address);
            
            switch (received_message.function_code) {
            case READ_COILS:
            case READ_DISC_INPUTS: {
                uint8_t byte_count = received_message.data[0];
                printf("%s data: ", (received_message.function_code == READ_COILS) ? "Coils" : "Inputs");
                for (int byte_idx = 0; byte_idx < byte_count; byte_idx++) {
                    uint8_t data_byte = received_message.data[1 + byte_idx];
                    printf("0x%02X ", data_byte);
                    // Show individual bits
                    printf("(");
                    for (int bit = 7; bit >= 0; bit--) {
                        printf("%d", (data_byte >> bit) & 1);
                    }
                    printf(") ");
                }
                printf("\n");
                break;
            }
            
            case READ_HOLD_REGS:
            case READ_INPUT_REGS: {
                uint8_t byte_count = received_message.data[0];
                printf("%s data: ", (received_message.function_code == READ_HOLD_REGS) ? "Holding Regs" : "Input Regs");
                for (int i = 0; i < byte_count; i += 2) {
                    uint16_t value = (received_message.data[1 + i] << 8) | received_message.data[2 + i];
                    printf("%d (0x%04X) ", value, value);
                }
                printf("\n");
                break;
            }
            
            case WRITE_SINGLE_COIL:
                printf("Coil write confirmed - Address: %d, Value: 0x%04X\n",
                       (received_message.data[0] << 8) | received_message.data[1],
                       (received_message.data[2] << 8) | received_message.data[3]);
                break;
                
            case WRITE_SINGLE_REG:
                printf("Register write confirmed - Address: %d, Value: %d\n",
                       (received_message.data[0] << 8) | received_message.data[1],
                       (received_message.data[2] << 8) | received_message.data[3]);
                break;
                
            case WRITE_MULTI_REGS:
                printf("Multiple registers write confirmed - Start: %d, Count: %d\n",
                       (received_message.data[0] << 8) | received_message.data[1],
                       (received_message.data[2] << 8) | received_message.data[3]);
                break;
                
            default:
                printf("Function %d response received\n", received_message.function_code);
            }
            
            waiting4response = 0;
            resend_count = 0;
            command_pending = false;
        } else {
            printf("✗ Invalid response received, raw data (%d bytes): ", rxDataLen);
            for (size_t i = 0; i < rxDataLen; i++) {
                printf("%02X ", uart_rxBuffer[i]);
            }
            printf("\n");
            waiting4response = 0;
            resend_count = 0;
            command_pending = false;
        }
        
        new_rxdata = 0;
        memset(uart_rxBuffer, 0, UART_BUFFER_SIZE);
        rxDataLen = 0;
        
        if (interactive_mode) {
            printf("\nmodbus> ");
            fflush(stdout);
        }
    }
    
    if (TIMER_ELAPSED_MS(resend_timer, 1000) && waiting4response) {
        resend_timer = TIMER_SET();
        resend_count++;
        printf("⚠ Timeout waiting for response, retry %d/5\n", resend_count);
        
        if (interactive_mode) {
            printf("modbus> ");
            fflush(stdout);
        }
    }
    
    if (resend_count >= 5) {
        printf("✗ Max retries reached, command failed\n");
        waiting4response = 0;
        resend_count = 0;
        command_pending = false;
        
        if (interactive_mode) {
            printf("\nmodbus> ");
            fflush(stdout);
        }
    }
}

// Main Modbus handler
static void modbus_handler(void) {
    // Try to read new data
    uart_read();
    
    if (waiting4response) {
        modbus_response();
    } else if (new_rxdata) {
        modbus_receive();
    }
}

// Check for keyboard input (non-blocking)
static int check_keyboard_input(void) {
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    
    return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
}

// Interactive command interface
static void interactive_interface(void) {
    static char input[MAX_CMD_LEN] = {0};
    static int input_pos = 0;
    static int cursor_pos = 0;
    
    if (check_keyboard_input() > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\n') {
                // Process command on Enter
                input[input_pos] = '\0';
                printf("\n");
                add_to_history(input);
                process_command(input);
                input_pos = 0;
                cursor_pos = 0;
                input[0] = '\0';
                printf("modbus> ");
                fflush(stdout);
            }
            else if (c == 27) { // ESC sequence
                char seq[3];
                if (read(STDIN_FILENO, &seq[0], 1) != 1) return;
                if (read(STDIN_FILENO, &seq[1], 1) != 1) return;
                
                if (seq[0] == '[') {
                    if (seq[1] == 'A') { // Up arrow
                        if (history_pos > 0) {
                            history_pos--;
                            // Clear current line
                            printf("\r\033[K");
                            printf("modbus> ");
                            strcpy(input, cmd_history[history_pos]);
                            input_pos = strlen(input);
                            cursor_pos = input_pos;
                            printf("%s", input);
                        }
                    }
                    else if (seq[1] == 'B') { // Down arrow
                        if (history_pos < history_count) {
                            history_pos++;
                            printf("\r\033[K");
                            printf("modbus> ");
                            if (history_pos == history_count) {
                                input[0] = '\0';
                                input_pos = 0;
                                cursor_pos = 0;
                            } else {
                                strcpy(input, cmd_history[history_pos]);
                                input_pos = strlen(input);
                                cursor_pos = input_pos;
                                printf("%s", input);
                            }
                        }
                    }
                    else if (seq[1] == 'C') { // Right arrow
                        if (cursor_pos < input_pos) {
                            cursor_pos++;
                            printf("\033[C");
                        }
                    }
                    else if (seq[1] == 'D') { // Left arrow
                        if (cursor_pos > 0) {
                            cursor_pos--;
                            printf("\033[D");
                        }
                    }
                }
            }
            else if (c == 127) { // Backspace
                if (cursor_pos > 0) {
                    // Move characters after cursor back by one position
                    memmove(&input[cursor_pos - 1], &input[cursor_pos], input_pos - cursor_pos);
                    cursor_pos--;
                    input_pos--;
                    input[input_pos] = '\0';
                    
                    // Redraw the line from cursor position
                    printf("\b\033[K%s", &input[cursor_pos]);
                    
                    // Move cursor back to correct position
                    if (cursor_pos < input_pos) {
                        printf("\033[%dD", input_pos - cursor_pos);
                    }
                }
            }
            else if (!iscntrl(c) && input_pos < MAX_CMD_LEN - 1) {
                // Insert character at cursor position
                memmove(&input[cursor_pos + 1], &input[cursor_pos], input_pos - cursor_pos);
                input[cursor_pos] = c;
                cursor_pos++;
                input_pos++;
                input[input_pos] = '\0';
                
                // Redraw the line from cursor position
                printf("\033[K%s", &input[cursor_pos - 1]);
                
                // Move cursor back to correct position
                if (cursor_pos < input_pos) {
                    printf("\033[%dD", input_pos - cursor_pos);
                }
            }
            fflush(stdout);
        }
    }
}

int main(int argc, char* argv[]) {
    const char* uart_device = "/dev/ttyACM0";
    bool run_interactive = true;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options] [device]\n", argv[0]);
            printf("Options:\n");
            printf("  -h, --help     Show this help\n");
            printf("  -i, --interactive  Interactive mode (default)\n");
            printf("  -s, --slave    Slave mode only\n");
            printf("  device         UART device (default: /dev/ttyACM0)\n");
            return 0;
        }
        else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--interactive") == 0) {
            run_interactive = true;
        }
        else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--slave") == 0) {
            run_interactive = false;
        }
        else if (argv[i][0] != '-') {
            uart_device = argv[i];
        }
    }
    
    printf("=== Modbus RTU Master/Slave ===\n");
    printf("Device: %s\n", uart_device);
    printf("Local Modbus ID: %d\n", MODBUS_ID);
    
    if (init_uart(uart_device) < 0) {
        return 1;
    }
    
    interactive_mode = run_interactive;
    if (interactive_mode) {
        enable_raw_mode();
        atexit(disable_raw_mode);
    }
    
    if (interactive_mode) {
        printf("\nInteractive Mode - Type 'help' for commands\n");
        print_help();
        printf("modbus> ");
        fflush(stdout);
    } else {
        printf("Slave Mode - Listening for Modbus commands...\n");
    }
    
    while (1) {
        modbus_handler();
        
        if (interactive_mode) {
            interactive_interface();
        }
        
        // Small delay to prevent CPU spinning
        usleep(1000); // 1ms
    }
    
    close(uart_fd);
    return 0;
}
