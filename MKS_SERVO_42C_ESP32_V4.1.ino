/**
 * @file MKS_SERVO42C_UART_Bridge.ino
 * @version 1.4.3
 * @author Lucas Barcaro (comments AI generated, if any error, please contact the author)
 * @date October 2025
 *
 * @brief Provides a full-featured, human-readable serial command interface
 * for the MKS SERVO42C closed-loop stepper driver via UART.
 *
 * This firmware runs on an ESP32 and acts as a serial bridge.
 * It translates intuitive string commands (e.g., "move 3200 30") into
 * the binary packet format required by the driver, including automatic
 * checksum calculation.
 *
 * This architecture is data-driven, using 'dictionaries' (const arrays
 * of structs) to map commands to their respective handlers, making the
 * code clean, scalable, and easy to maintain.
 *
 * @features
 * - Human-readable command parser (e.g., "enable", "set_kp 1500")
 * - Automatic 8-bit checksum calculation
 * - Full implementation of all commands from the V1.1.2 manual
 * - Data-driven routing for simple and single/multi-byte arg commands
 * - Robust error handling with `int8_t` status codes
 * - Interactive, multi-level 'help' menu
 * - Non-blocking serial read for driver responses
 */

// --- UART2 (Driver) Settings ---
#define DRIVER_BAUD_RATE 38400                // Default baud for MKS SERVO42C
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17

// --- Receive Buffer Settings ---
#define RX_BUFFER_SIZE 64                     // Max expected bytes in a single driver response
byte rxBuffer[RX_BUFFER_SIZE];
int rxBufferIndex                 = 0;
unsigned long lastByteTime        = 0;
const long messageTimeout         = 20;       // 20ms of silence = end of message

// --- Driver Protocol Constants ---
const byte DRIVER_ADDR            = 0xE0;     // Default driver address
const byte MAX_SPEED_VALUE        = 0x7F;     // 127 (7-bit value)
const int MAX_TORQUE_VALUE        = 0x0480;   // 1152 (from manual)

// =================================================================
// STATUS CODES
// =================================================================
/**
 * @brief Defines the return codes for command parsing functions.
 * Provides detailed error feedback instead of a simple true/false.
 */
const int8_t STATUS_OK            =  0; // Command successful and sent
const int8_t ERR_UNKNOWN_COMMAND  = -1; // Command string was not recognized
const int8_t ERR_INVALID_ARGS     = -2; // Missing, empty, or malformed arguments
const int8_t ERR_ARG_OUT_OF_RANGE = -3; // Argument value is outside the allowed range

// =================================================================
// COMMAND DICTIONARIES (Data-Driven Routing)
// =================================================================

/**
 * @struct SimpleCommand
 * @brief Dictionary for commands that have NO user arguments
 * (e.g., "enable", "read_encoder").
 */
struct SimpleCommand {
  const char* name;                     // The human-readable command
  byte bytes[3];                        // The command packet bytes (excluding Addr/Checksum)
  int length;                           // The number of bytes in the packet
};

const SimpleCommand simpleCommands[] = {
  // Control
  {"enable",       {0xF3, 0x01}, 2},
  {"disable",      {0xF3, 0x00}, 2},
  {"stop",         {0xF7},       1},
  // Feedback
  {"read_encoder", {0x30},       1},
  {"read_pulses",  {0x33},       1},
  {"read_error_angle", {0x39},   1},
  {"read_en_status", {0x3A},     1},
  {"release_protection", {0x3D}, 1},
  {"read_protection_state", {0x3E},1},
  // Config
  {"calibrate",    {0x80, 0x00}, 2},
  {"restore_defaults", {0x3F},   1},
  {"set_zero",     {0x91, 0x00}, 2},
  {"goto_zero",    {0x94, 0x00}, 2},
  {"save_spin_state", {0xFF, 0xC8}, 2},
  {"clear_spin_state", {0xFF, 0xCA}, 2}
};
const int numSimpleCommands = sizeof(simpleCommands) / sizeof(simpleCommands[0]);

/**
 * @struct SingleByteArgCommand
 * @brief Dictionary for commands with ONE 1-byte argument
 * (e.g., "set_baud 4").
 */
struct SingleByteArgCommand {
  const char* name;
  byte functionCode;
  int minVal;
  int maxVal;
  const char* errorMsg;         // Help text for error reporting
};
const SingleByteArgCommand singleByteCommands[] = {
  {"set_motor_type", 0x81, 0, 1, "type (0=0.9deg, 1=1.8deg)"},
  {"set_work_mode",  0x82, 0, 2, "mode (0=Open, 1=vFOC, 2=UART)"},
  {"set_current",    0x83, 0, 15, "current code (0-15)"},
  {"set_subdivision",0x84, 1, 255, "subdivision (1-255)"},
  {"set_en_level",   0x85, 0, 2, "level (0=L, 1=H, 2=Hold)"},
  {"set_direction",  0x86, 0, 1, "direction (0=CW, 1=CCW)"},
  {"set_screen_sleep", 0x87, 0, 1, "sleep (0=Off, 1=On)"},
  {"set_protection", 0x88, 0, 1, "protection (0=Off, 1=On)"},
  {"set_interpolation", 0x89, 0, 1, "interpolation (0=Off, 1=On)"},
  {"set_baud",       0x8A, 1, 6, "baud index (1=9600... 6=115200)"},
  {"set_address",    0x8B, 0, 9, "address index (0=e0... 9=e9)"},
  {"set_zero_mode",  0x90, 0, 2, "mode (0=Disable, 1=Dir, 2=Near)"},
  {"set_zero_speed", 0x92, 0, 4, "speed (0=fast... 4=slow)"},
  {"set_zero_dir",   0x93, 0, 1, "direction (0=CW, 1=CCW)"}
};
const int numSingleByteCommands = sizeof(singleByteCommands) / sizeof(singleByteCommands[0]);

/**
 * @struct TwoByteArgCommand
 * @brief Dictionary for commands with ONE 2-byte (uint16_t) argument
 * (e.g., "set_kp 1500").
 */
struct TwoByteArgCommand {
  const char* name;
  byte functionCode;
  long minVal;
  long maxVal;
  const char* errorMsg;         // Help text for error reporting
};
const TwoByteArgCommand twoByteCommands[] = {
  {"set_kp",        0xA1, 0, 65535, "Kp (0-65535)"},
  {"set_ki",        0xA2, 0, 65535, "Ki (0-65535)"},
  {"set_kd",        0xA3, 0, 65535, "Kd (0-65535)"},
  {"set_acc",       0xA4, 0, 65535, "ACC (0-65535)"},
  {"set_maxtorque", 0xA5, 0, MAX_TORQUE_VALUE, "Max Torque (0-1152)"}
};
const int numTwoByteCommands = sizeof(twoByteCommands) / sizeof(twoByteCommands[0]);


// =================================================================
// SETUP & MAIN LOOP
// =================================================================

/**
 * @brief Initializes serial ports (to PC and to Driver)
 * and prints the welcome message.
 */
void setup() {
  // Serial to PC (for user interface)
  Serial.begin(115200);
  while (!Serial);

  // Serial to MKS Driver (Hardware UART2)
  Serial2.begin(DRIVER_BAUD_RATE, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  Serial.println("\n--- MKS Controller (v4.3 - Clean Comments) ---");
  Serial.println("Type 'help' for a list of commands.");
}

/**
 * @brief Main execution loop.
 * 1. Checks for new commands from the user (PC Serial).
 * 2. Checks for new responses from the driver (Driver Serial2).
 */
void loop() {
  // Check for user input from the Serial Monitor
  if (Serial.available()) {
    String rawInput = Serial.readStringUntil('\n');
    rawInput.trim();
    if (rawInput.length() > 0) {
      Serial.print("\n> ");             // Echo the command
      Serial.println(rawInput);
      parseAndSendCommand(rawInput);    // Process the command
    }
  }

  // Check for any response data from the driver
  handleDriverResponses();
}

// =================================================================
// MASTER COMMAND ROUTER
// =================================================================

/**
 * @brief Parses the user's raw input string and routes it to the correct handler.
 * This is the main "switchboard" of the application. It attempts
 * to find a match in the following order:
 * 1. Custom Handlers (help, move, spin)
 * 2. Single-Byte Argument Dictionary
 * 3. Two-Byte Argument Dictionary
 * 4. Simple (No-Argument) Dictionary
 *
 * @param line The raw command string (e.g., "move 3200 30" or "help 1").
 */
void parseAndSendCommand(String line) {
  line.toLowerCase();

  // Split the line into a "command" and "arguments"
  String command = "";
  String args = "";
  int firstSpace = line.indexOf(' ');

  if (firstSpace == -1) {
    command = line;                           // No spaces, so no args (e.g., "enable")
  } else {
    command = line.substring(0, firstSpace);  // "move"
    args = line.substring(firstSpace + 1);    // "3200 30"
    args.trim();
  }

  int8_t returnVal = ERR_UNKNOWN_COMMAND;     // Assume failure until a handler succeeds

  // --- Routing Block ---

  // 1. Handle commands with custom logic (help, move, spin)
  if (command == "help") {
    returnVal = handleHelpCommand(args);
  } 
  else if (command == "move") {
    returnVal = handleMoveCommand(args);
  } 
  else if (command == "spin") {
    returnVal = handleSpinCommand(args);
  } 
  // 2. If not a custom command, search the data-driven dictionaries
  else {
    // Try searching 1-byte-arg commands (e.g., "set_baud 4")
    returnVal = findAndSendSingleByteArgCommand(command, args);
    
    // If not found, try 2-byte-arg commands (e.g., "set_kp 1500")
    if (returnVal == ERR_UNKNOWN_COMMAND) {
      returnVal = findAndSendTwoByteArgCommand(command, args);
    }

    // If still not found, try simple (no-arg) commands (e.g., "enable")
    if (returnVal == ERR_UNKNOWN_COMMAND) {
      returnVal = findAndSendSimpleCommand(command);
    }
  }

  // 3. Final Error Feedback
  if (returnVal == ERR_UNKNOWN_COMMAND) {
    // Only print "not recognized" if no handler (including helpers)
    // has already printed a more specific error.
    Serial.println("Error: Command '" + command + "' not recognized.");
  }
}

// =================================================================
// COMMAND DICTIONARY "FINDER" FUNCTIONS (Routing Logic)
// =================================================================

/**
 * @brief Searches the 1-byte-arg dictionary and calls the generic handler.
 * @param command The command name to find (e.g., "set_baud").
 * @param args The argument string (e.g., "4").
 * @return int8_t Status code (STATUS_OK, ERR_INVALID_ARGS, etc.)
 */
int8_t findAndSendSingleByteArgCommand(String command, String args) {
  int8_t returnVal = ERR_UNKNOWN_COMMAND;
  for (int i = 0; i < numSingleByteCommands; i++) {
    if (command == singleByteCommands[i].name) {
      // Found! Call the generic handler with the dictionary data.
      returnVal = handleSingleByteArgCommand(
        args,
        singleByteCommands[i].functionCode,
        singleByteCommands[i].minVal,
        singleByteCommands[i].maxVal,
        singleByteCommands[i].errorMsg
      );
      break; // Exit loop
    }
  }
  return returnVal;
}

/**
 * @brief Searches the 2-byte-arg dictionary and calls the generic handler.
 * @param command The command name to find (e.g., "set_kp").
 * @param args The argument string (e.g., "1500").
 * @return int8_t Status code (STATUS_OK, ERR_INVALID_ARGS, etc.)
 */
int8_t findAndSendTwoByteArgCommand(String command, String args) {
  int8_t returnVal = ERR_UNKNOWN_COMMAND;
  for (int i = 0; i < numTwoByteCommands; i++) {
    if (command == twoByteCommands[i].name) {
      // Found! Call the generic handler with the dictionary data.
      returnVal = handleTwoByteArgCommand(
        args,
        twoByteCommands[i].functionCode,
        twoByteCommands[i].minVal,
        twoByteCommands[i].maxVal,
        twoByteCommands[i].errorMsg
      );
      break; // Exit loop
    }
  }
  return returnVal;
}

/**
 * @brief Searches the simple command dictionary and sends the packet.
 * @param command The name of the command to find (e.g., "enable").
 * @return int8_t Returns STATUS_OK if found, or ERR_UNKNOWN_COMMAND if not.
 */
int8_t findAndSendSimpleCommand(String command) {
  int8_t returnVal = ERR_UNKNOWN_COMMAND; 

  for (int i = 0; i < numSimpleCommands; i++) {
    if (command == simpleCommands[i].name) {
      // 1. Command found! Assemble the packet: [ADDR] + [command bytes]
      int cmdLength = simpleCommands[i].length;
      byte packet[1 + cmdLength];                 // 1 for the ADDR
      
      packet[0] = DRIVER_ADDR;
      // 2. Copy the command bytes (e.g., {0xF3, 0x01}) into the packet
      memcpy(&packet[1], simpleCommands[i].bytes, cmdLength);
      
      // 3. Send
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK; 
      break;                                      // Exit the 'for' loop
    }
  }
  return returnVal;
}

// =================================================================
// COMMAND LOGIC HANDLERS (Custom & Generic)
// =================================================================

/**
 * @brief Handles the 'help' command's interactive menu.
 * @param args The argument string (e.g., "" or "1" or "5").
 * @return int8_t Status code (STATUS_OK or ERR_ARG_OUT_OF_RANGE).
 */
int8_t handleHelpCommand(String args) {
    int8_t returnVal = STATUS_OK;
    if (args.length() == 0) {
        printHelp_Main(); // Show main menu
    } else {
        int category = args.toInt();
        switch (category) {
            case 1: printHelp_Motion(); break;
            case 2: printHelp_Homing(); break;
            case 3: printHelp_Feedback(); break;
            case 4: printHelp_PID(); break;
            case 5: printHelp_Config(); break;
            default:
              Serial.println("Error: Invalid help category. Must be 1-5.");
              returnVal = ERR_ARG_OUT_OF_RANGE;
              break;
        }
    }
    return returnVal;
}

/**
 * @brief Parses arguments for the 'move' command (custom logic).
 * @param args The string containing arguments (e.g., "3200 30").
 * @return int8_t A status code (STATUS_OK, ERR_INVALID_ARGS, etc.)
 */
int8_t handleMoveCommand(String args) {
  int8_t returnVal = ERR_INVALID_ARGS; 
  int spaceIndex = args.indexOf(' ');

  if (spaceIndex == -1) {
    Serial.println("Error: 'move' requires <pulses> and <speed> (e.g., move 3200 30)");
  } else {
    // 1. Parse arguments
    long pulses = args.substring(0, spaceIndex).toInt();
    int speed_raw = args.substring(spaceIndex + 1).toInt();
    byte speed_val = (byte)abs(speed_raw);

    // 2. Validate arguments
    if (speed_val > MAX_SPEED_VALUE) {
      Serial.println("Error: Speed must be between -" + String(MAX_SPEED_VALUE) + " and " + String(MAX_SPEED_VALUE));
      returnVal = ERR_ARG_OUT_OF_RANGE;
    } else {
      // 3. Assemble packet
      byte packet[7]; 
      packet[0] = DRIVER_ADDR;
      packet[1] = 0xFD;                         // 'Run by serial' command
      
      byte direction = (pulses < 0) ? 1 : 0;
      pulses = abs(pulses);
      packet[2] = (direction << 7) | speed_val; // VAL byte

      // Split 'long' pulses into 4 bytes (MSB first)
      packet[3] = (pulses >> 24) & 0xFF; 
      packet[4] = (pulses >> 16) & 0xFF;
      packet[5] = (pulses >> 8) & 0xFF;
      packet[6] = pulses & 0xFF;         
      
      // 4. Send
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK; 
    }
  }
  return returnVal;
}

/**
 * @brief Parses arguments for the 'spin' command (custom logic).
 * @param args The string containing the speed (e.g., "10" or "-10").
 * @return int8_t A status code (STATUS_OK, ERR_INVALID_ARGS, etc.)
 */
int8_t handleSpinCommand(String args) {
  int8_t returnVal = ERR_INVALID_ARGS;
  int speed_raw = args.toInt();
  byte speed_val = (byte)abs(speed_raw);

  // 1. Validate arguments
  if (speed_raw == 0) {
    Serial.println("Error: 'spin' requires a non-zero speed (e.g., spin 10)");
  } 
  else if (speed_val > MAX_SPEED_VALUE) {
    Serial.println("Error: Speed must be between -" + String(MAX_SPEED_VALUE) + " and " + String(MAX_SPEED_VALUE));
    returnVal = ERR_ARG_OUT_OF_RANGE;
  } 
  else {
    // 2. Assemble packet
    byte direction = (speed_raw < 0) ? 1 : 0;
    byte val = (direction << 7) | speed_val;      // VAL byte
    
    byte packet[] = {DRIVER_ADDR, 0xF6, val};     // 'Constant speed' command
    
    // 3. Send
    sendPacketWithChecksum(packet, sizeof(packet));
    returnVal = STATUS_OK;
  }
  return returnVal;
}

/**
 * @brief Generic handler for all "Set" commands that take a single byte argument.
 * @param args The user's argument string (must contain one number).
 * @param functionCode The command's function byte (e.g., 0x81).
 * @param minVal The minimum allowed value for the argument.
 * @param maxVal The maximum allowed value for the argument.
 * @param errorMsg A help string (e.g., "type (0-1)").
 * @return int8_t A status code (STATUS_OK, ERR_INVALID_ARGS, etc.)
 */
int8_t handleSingleByteArgCommand(String args, byte functionCode, int minVal, int maxVal, const char* errorMsg) {
  int8_t returnVal = ERR_INVALID_ARGS;
  
  // 1. Validate arguments
  if (args.length() == 0) {
    Serial.println("Error: Missing argument. Usage: <cmd> " + String(errorMsg));
  } else {
    int arg_val_raw = args.toInt();
    if (arg_val_raw < minVal || arg_val_raw > maxVal) {
      Serial.println("Error: Argument out of range. Must be " + String(errorMsg));
      returnVal = ERR_ARG_OUT_OF_RANGE;
    } else {
      // 2. Assemble packet
      byte arg_val = (byte)arg_val_raw;
      byte packet[] = {DRIVER_ADDR, functionCode, arg_val};
      
      // 3. Send
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK;
    }
  }
  return returnVal;
}

/**
 * @brief Generic handler for all "Set" commands that take two bytes (uint16_t).
 * @param args The user's argument string (must contain one number).
 * @param functionCode The command's function byte (e.g., 0xA1).
 * @param minVal The minimum allowed value for the argument.
 * @param maxVal The maximum allowed value for the argument.
 * @param errorMsg A help string (e.g., "Kp (0-65535)").
 * @return int8_t A status code (STATUS_OK, ERR_INVALID_ARGS, etc.)
 */
int8_t handleTwoByteArgCommand(String args, byte functionCode, long minVal, long maxVal, const char* errorMsg) {
  int8_t returnVal = ERR_INVALID_ARGS;
  
  // 1. Validate arguments
  if (args.length() == 0) {
    Serial.println("Error: Missing argument. Usage: <cmd> " + String(errorMsg));
  } else {
    long arg_val_raw = args.toInt(); // Use long to check 0-65535
    if (arg_val_raw < minVal || arg_val_raw > maxVal) {
      Serial.println("Error: Argument out of range. Must be " + String(errorMsg));
      returnVal = ERR_ARG_OUT_OF_RANGE;
    } else {
      // 2. Assemble packet
      uint16_t arg_val = (uint16_t)arg_val_raw;
      
      byte packet[4]; // Addr, Func, Arg_Hi, Arg_Lo
      packet[0] = DRIVER_ADDR;
      packet[1] = functionCode;
      packet[2] = (arg_val >> 8) & 0xFF;      // High Byte (MSB)
      packet[3] = arg_val & 0xFF;             // Low Byte (LSB)
      
      // 3. Send
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK;
    }
  }
  return returnVal;
}


// =================================================================
// AUXILIARY FUNCTIONS (Communication & UI)
// =================================================================

/**
 * @brief Sends a data packet to the driver, calculating and appending the checksum.
 * @param packetData The byte array of the command (e.g., {0xE0, 0xF3, 0x01}).
 * @param dataLength The total number of bytes in the packetData array.
 */
void sendPacketWithChecksum(byte* packetData, int dataLength) {
  byte checksum = 0;
  
  Serial.print("Sending Packet:   [ ");
  for (int i = 0; i < dataLength; i++) {
    checksum += packetData[i];                    // Additive 8-bit checksum
    
    // Print the byte in HEX format
    Serial.print("0x");
    if (packetData[i] < 0x10) Serial.print("0");  // Add leading zero
    Serial.print(packetData[i], HEX);
    Serial.print(" ");
  }

  // Append the calculated checksum
  Serial.print("0x");
  if (checksum < 0x10) Serial.print("0");
  Serial.print(checksum, HEX);
  Serial.println(" ] (Checksum)");

  // Send the data and the checksum byte to the driver
  Serial2.write(packetData, dataLength);
  Serial2.write(checksum);
}

/**
 * @brief Handles multi-byte responses from the driver (non-blocking).
 * Collects bytes from Serial2 into rxBuffer until a 'messageTimeout'
 * (20ms) of silence occurs, then prints the complete message.
 */
void handleDriverResponses() {
  // 1. Read all available bytes into the buffer
  while (Serial2.available()) {
    if (rxBufferIndex < RX_BUFFER_SIZE) {
      // Add byte to buffer and reset the timeout timer
      rxBuffer[rxBufferIndex] = Serial2.read();
      rxBufferIndex++;
      lastByteTime = millis();
    } else {
      // Buffer overflow, discard the byte but reset timer
      Serial2.read();
      lastByteTime = millis();
    }
  }

  // 2. Check if the message timeout has expired
  if (rxBufferIndex > 0 && (millis() - lastByteTime > messageTimeout)) {
    // Message is complete, print it
    Serial.print("Driver Response <- [ ");
    for (int i = 0; i < rxBufferIndex; i++) {
      Serial.print("0x");
      if (rxBuffer[i] < 0x10) Serial.print("0");
      Serial.print(rxBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println("]");
    
    // 3. Reset the buffer for the next message
    rxBufferIndex = 0; 
  }
}

// =================================================================
// HELP MENU PRINTING FUNCTIONS
// =================================================================

/**
 * @brief Prints the MAIN help menu (categories).
 */
void printHelp_Main() {
  Serial.println("--- MKS Controller Help Menu ---");
  Serial.println("Usage: help <category_number>");
  Serial.println(" 1. Motion Commands");
  Serial.println(" 2. Homing (Zero) Commands");
  Serial.println(" 3. Read (Feedback) Commands");
  Serial.println(" 4. Configuration (PID) Commands");
  Serial.println(" 5. Configuration (Driver) Commands");
  Serial.println("--------------------------------");
}

/**
 * @brief Prints help for Category 1.
 */
void printHelp_Motion() {
  Serial.println("--- 1. Motion Commands ---");
  Serial.println("enable                 : Enables the motor");
  Serial.println("disable                : Disables the motor");
  Serial.println("spin <speed>           : Spins continuously (speed: -127 to 127)");
  Serial.println("stop                   : Stops the 'spin' motion");
  Serial.println("move <pulses> <speed>  : Moves N pulses (speed: 0-127, pulses: -/+ long)");
  Serial.println("set_direction <0|1>    : Sets motor direction (0=CW, 1=CCW)");
  Serial.println("set_acc <0-65535>      : Sets acceleration");
  Serial.println("set_maxtorque <0-1152> : Sets the maximum torque");
}

/**
 * @brief Prints help for Category 2.
 */
void printHelp_Homing() {
  Serial.println("--- 2. Homing (Zero) Commands ---");
  Serial.println("set_zero               : Sets the current position as 0");
  Serial.println("goto_zero              : Moves to the 0 position");
  Serial.println("set_zero_mode <0-2>    : Sets homing mode (0=Off, 1=Dir, 2=Near)");
  Serial.println("set_zero_speed <0-4>   : Sets homing speed (0=fast, 4=slow)");
  Serial.println("set_zero_dir <0|1>     : Sets homing direction (0=CW, 1=CCW)");
}

/**
 * @brief Prints help for Category 3.
 */
void printHelp_Feedback() {
  Serial.println("--- 3. Read (Feedback) Commands ---");
  Serial.println("read_encoder           : Reads the current encoder position");
  Serial.println("read_pulses            : Reads the received pulse counter");
  Serial.println("read_error_angle       : Reads the current angle error");
  Serial.println("read_en_status         : Reads the Enable pin status");
  Serial.println("read_protection_state  : Reads protection status (1=locked, 2=ok)");
  Serial.println("release_protection     : Releases motor from a locked-rotor fault");
}

/**
 * @brief Prints help for Category 4.
 */
void printHelp_PID() {
  Serial.println("--- 4. Configuration (PID) Commands ---");
  Serial.println("set_kp <0-65535>       : Sets Proportional gain (Kp)");
  Serial.println("set_ki <0-65535>       : Sets Integral gain (Ki)");
  Serial.println("set_kd <0-65535>       : Sets Derivative gain (Kd)");
}

/**
 * @brief Prints help for Category 5.
 */
void printHelp_Config() {
  Serial.println("--- 5. Configuration (Driver) Commands ---");
  Serial.println("calibrate              : Starts motor calibration (must be unloaded)");
  Serial.println("restore_defaults       : Restores factory settings (requires reboot)");
  Serial.println("set_motor_type <0|1>   : Sets motor type (0=0.9deg, 1=1.8deg)");
  Serial.println("set_work_mode <0-2>    : Sets mode (0=Open, 1=vFOC/Pulse, 2=UART)");
  Serial.println("set_current <0-15>     : Sets current code (0=200mA... 15=3000mA)");
  Serial.println("set_subdivision <1-255> : Sets microstepping (1-255)");
  Serial.println("set_en_level <0-2>     : Sets EN pin logic (0=L, 1=H, 2=Hold/Always_On)");
  Serial.println("set_screen_sleep <0|1> : Enables OLED sleep (0=Off, 1=On)");
  Serial.println("set_protection <0|1>   : Enables locked-rotor protection (0=Off, 1=On)");
  Serial.println("set_interpolation <0|1> : Enables subdivision interpolation (0=Off, 1=On)");
  Serial.println("set_baud <1-6>         : Sets baud rate index (1=9600... 6=115200)");
  Serial.println("set_address <0-9>      : Sets address index (0=e0... 9=e9)");
  Serial.println("save_spin_state        : Saves 'spin' command to run on boot");
  Serial.println("clear_spin_state       : Clears 'spin' command from boot");
} 