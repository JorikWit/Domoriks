# Modbus RTU Master/Slave Tool

A command-line tool for Modbus RTU communication that can act as both master and slave. Supports various Modbus functions and provides an interactive interface with command history.

## Building

### For Linux ARM64 (HASS)
```bash
aarch64-linux-gnu-gcc -static -o modbus modbus.c -lrt
```

### For Linux x86_64
```bash
gcc -o modbus modbus.c -lrt
```

## Usage

```bash
./modbus [options] [device]
```

### Options
- `-h, --help`: Show help message
- `-i, --interactive`: Interactive mode (default)
- `-s, --slave`: Slave mode only
- `device`: UART device (default: /dev/ttyACM0)

## Commands

### Reading Operations
- `rc <slave> <addr> <count>`: Read Coils
  ```bash
  rc 1 0 16    # Read 16 coils from slave 1, starting at address 0
  ```

- `ri <slave> <addr> <count>`: Read Discrete Inputs
  ```bash
  ri 2 0 8     # Read 8 discrete inputs from slave 2, starting at address 0
  ```

- `rh <slave> <addr> <count>`: Read Holding Registers
  ```bash
  rh 1 0 10    # Read 10 holding registers from slave 1, starting at address 0
  ```

- `rr <slave> <addr> <count>`: Read Input Registers
  ```bash
  rr 1 0 4     # Read 4 input registers from slave 1, starting at address 0
  ```

### Writing Operations
- `wc <slave> <addr> <value>`: Write Single Coil
  ```bash
  wc 1 0 1     # Turn ON coil 0 on slave 1
  wc 1 0 0     # Turn OFF coil 0 on slave 1
  wc 1 0 2     # Toggle coil 0 on slave 1
  ```

- `wr <slave> <addr> <value>`: Write Single Register
  ```bash
  wr 1 0 1234  # Write value 1234 to register 0 on slave 1
  ```

- `wmc <slave> <addr> <hex_string>`: Write Multiple Coils
  ```bash
  wmc 1 0 FF   # Write 8 coils ON on slave 1
  wmc 1 0 F0   # Write first 4 coils ON, next 4 OFF on slave 1
  ```

- `wmr <slave> <addr> <values...>`: Write Multiple Registers
  ```bash
  wmr 1 0 1234 5678    # Write values 1234 and 5678 to registers 0 and 1
  ```

### Other Commands
- `status`: Show current coil/register status
- `setid <id>`: Set local Modbus ID (1-247)
- `help`: Show help message
- `quit` or `exit`: Exit program

## Navigation
- Use Up/Down arrows to navigate through command history
- Use Left/Right arrows to move cursor within current command
- Use Backspace to delete characters

## Serial Port Settings
- Baud Rate: 115200
- Data Bits: 8
- Stop Bits: 1
- Parity: None
- Flow Control: None
