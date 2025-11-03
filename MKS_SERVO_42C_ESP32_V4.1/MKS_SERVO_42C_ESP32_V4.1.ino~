/**
 * @file MKS_SERVO42C_UART_Bridge.ino
 * @version 1.5.0 (Stable Pulse Homing)
 * @author Lucas Barcaro (comments AI generated, if any error, please contact the author)
 * @date October 2025
 *
 * @brief This version provides a full-featured, human-readable serial command
 * interface for the MKS SERVO42C closed-loop stepper driver via UART.
 *
 * @details This firmware runs on an ESP32 and acts as an advanced serial bridge.
 * It translates intuitive string commands (e.g., "move 3200 30") into
 * the binary packet format required by the driver, including automatic
 * checksum calculation.
 *
 * This version uses the 'read_pulses' (0x33) command as the
 * non-circular 32-bit position tracker, which correctly
 * matches the 'move' (0xFD) command's units.
 *
 * The software-based 'set_home' and 'go_home' now
 * correctly track the multi-rotation absolute pulse position.
 */

// --- UART2 (Driver) Settings ---
#define DRIVER_BAUD_RATE 38400
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17

// --- Receive Buffer Settings ---
#define RX_BUFFER_SIZE 64
byte rxBuffer[RX_BUFFER_SIZE];
int rxBufferIndex = 0;
unsigned long lastByteTime = 0;
const long messageTimeout = 20; // Use 20ms for a safer buffer capture

// --- Driver Protocol Constants ---
const byte DRIVER_ADDR = 0xE0;
const byte MAX_SPEED_VALUE = 0x7F;
const int MAX_TORQUE_VALUE = 0x0480;

// =================================================================
// STATUS CODES
// =================================================================
/**
 * @brief Defines the return codes for command parsing functions.
 * Provides detailed error feedback instead of a simple true/false.
 */
const int8_t STATUS_OK               =  0; // Command successful and sent
const int8_t ERR_UNKNOWN_COMMAND   = -1; // Command string was not recognized
const int8_t ERR_INVALID_ARGS      = -2; // Missing, empty, or malformed arguments
const int8_t ERR_ARG_OUT_OF_RANGE  = -3; // Argument value is outside the allowed range

// =================================================================
// STATE MACHINE & ABSOLUTE POSITION GLOBALS
// =================================================================

/**
 * @brief Defines the pending action to be executed when a
 * 'read_pulses' response is received.
 */
enum PendingAction {
  NONE,       // Not waiting for any data
  SET_HOME,   // Waiting for pulse data to set the home offset
  GO_HOME     // Waiting for pulse data to calculate return move
};
PendingAction currentAction = NONE;

/**
 * @brief Stores the user-defined absolute "home" position (in pulses).
 * This is the value of the 'read_pulses' counter when 'set_home' is called.
 */
int32_t absoluteHomeOffset_pulses = 0;

/**
 * @brief Stores the last known absolute position from the pulse counter.
 */
int32_t currentAbsolutePosition_pulses = 0;

/**
 * @brief Flag to indicate if 'absoluteHomeOffset_pulses' holds a valid tare.
 */
bool positionIsKnown = false;

/**
 * @brief Stores the speed for the 'go_home' command, as it's
 * needed *after* the encoder response is received.
 */
int pendingGoHomeSpeed = 0;


// =================================================================
// COMMAND DICTIONARIES (Data-Driven Routing)
// =================================================================

/**
 * @struct SimpleCommand
 * @brief Dictionary for commands that have NO user arguments
 * (e.g., "enable", "read_pulses").
 */
struct SimpleCommand {
  const char* name;
  byte bytes[3];
  int length;
};
const SimpleCommand simpleCommands[] = {
  {"enable",       {0xF3, 0x01}, 2},
  {"disable",      {0xF3, 0x00}, 2},
  {"stop",         {0xF7},       1},
  {"read_encoder", {0x30},       1},
  {"read_pulses",  {0x33},       1},
  {"read_error_angle", {0x39},   1},
  {"read_en_status", {0x3A},     1},
  {"release_protection", {0x3D}, 1},
  {"read_protection_state", {0x3E},1},
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
  const char* errorMsg; // Help text for error reporting
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
  const char* errorMsg; // Help text for error reporting
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

  // Update version in welcome message
  Serial.println("\n--- MKS Controller (v1.5.0 - Correct Pulse Tracking) ---");
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
      Serial.print("\n> "); // Echo the command
      Serial.println(rawInput);
      parseAndSendCommand(rawInput); // Process the command
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
 * 1. Custom Handlers (help, move, spin, set_home, go_home)
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
    command = line; // No spaces, so no args (e.g., "enable")
  } else {
    command = line.substring(0, firstSpace); // "move"
    args = line.substring(firstSpace + 1);    // "3200 30"
    args.trim();
  }

  int8_t returnVal = ERR_UNKNOWN_COMMAND; // Assume failure until a handler succeeds

  // --- Routing Block ---

  // 1. Handle commands with custom logic
  if (command == "help") {
    returnVal = handleHelpCommand(args);
  } 
  else if (command == "move") {
    returnVal = handleMoveCommand(args);
  } 
  else if (command == "spin") {
    returnVal = handleSpinCommand(args);
  }
  else if (command == "set_home") {
    returnVal = handleSetHome();
  }
  else if (command == "go_home") {
    returnVal = handleGoHome(args);
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
      int cmdLength = simpleCommands[i].length;
      byte packet[1 + cmdLength]; 
      packet[0] = DRIVER_ADDR;
      memcpy(&packet[1], simpleCommands[i].bytes, cmdLength);
      
      // Special case: 'read_pulses' triggers the state machine
      // Don't allow a manual read if another action is waiting
      if (strcmp(simpleCommands[i].name, "read_pulses") == 0) {
        if (currentAction != NONE) {
           Serial.println("Error: Another action is pending.");
           return ERR_INVALID_ARGS; // "Busy"
        }
      }
      
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK; 
      break; // Exit the 'for' loop
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
      // 3. Send Packet
      sendMovePacket(pulses, speed_val);
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
    byte val = (direction << 7) | speed_val; // VAL byte
    
    byte packet[] = {DRIVER_ADDR, 0xF6, val}; // 'Constant speed' command
    
    // 3. Send
    sendPacketWithChecksum(packet, sizeof(packet));
    returnVal = STATUS_OK;
  }
  return returnVal;
}

/**
 * @brief Handler for 'set_home'. Triggers a pulse read.
 * @return int8_t Status code.
 */
int8_t handleSetHome() {
  int8_t returnVal = STATUS_OK;
  if (currentAction != NONE) {
    Serial.println("Error: Cannot set home, another action is pending.");
    returnVal = ERR_INVALID_ARGS; // "Busy"
  } else {
    Serial.println("Action: Requesting current pulse count to set as home...");
    currentAction = SET_HOME;
    triggerPulsesRead(); // <-- Use 'read_pulses'
  }
  return returnVal;
}

/**
 * @brief Handler for 'go_home'. Triggers a pulse read.
 * @param args The argument string (e.g. "30").
 * @return int8_t Status code.
 */
int8_t handleGoHome(String args) {
  int8_t returnVal = ERR_INVALID_ARGS;
  
  if (currentAction != NONE) {
    Serial.println("Error: Cannot go home, another action is pending.");
  } else if (!positionIsKnown) {
    Serial.println("Error: Home position is not set. Use 'set_home' first.");
  } else {
    int speed_raw = args.toInt();
    byte speed_val = (byte)abs(speed_raw);
    
    if (speed_raw == 0) {
      Serial.println("Error: 'go_home' requires a non-zero speed (e.g., go_home 30)");
    }
    else if (speed_val > MAX_SPEED_VALUE) {
      Serial.println("Error: Speed must be between 1 and " + String(MAX_SPEED_VALUE));
      returnVal = ERR_ARG_OUT_OF_RANGE;
    }
    else {
      Serial.println("Action: Requesting current pulse count to calculate return move...");
      pendingGoHomeSpeed = speed_val;
      currentAction = GO_HOME;
      triggerPulsesRead(); // <-- Use 'read_pulses'
      returnVal = STATUS_OK;
    }
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
  
  if (args.length() == 0) {
    Serial.println("Error: Missing argument. Usage: <cmd> " + String(errorMsg));
  } else {
    int arg_val_raw = args.toInt();
    if (arg_val_raw < minVal || arg_val_raw > maxVal) {
      Serial.println("Error: Argument out of range. Must be " + String(errorMsg));
      returnVal = ERR_ARG_OUT_OF_RANGE;
    } else {
      byte arg_val = (byte)arg_val_raw;
      byte packet[] = {DRIVER_ADDR, functionCode, arg_val};
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
  
  if (args.length() == 0) {
    Serial.println("Error: Missing argument. Usage: <cmd> " + String(errorMsg));
  } else {
    long arg_val_raw = args.toInt();
    if (arg_val_raw < minVal || arg_val_raw > maxVal) {
      Serial.println("Error: Argument out of range. Must be " + String(errorMsg));
      returnVal = ERR_ARG_OUT_OF_RANGE;
    } else {
      uint16_t arg_val = (uint16_t)arg_val_raw;
      byte packet[4]; 
      packet[0] = DRIVER_ADDR;
      packet[1] = functionCode;
      packet[2] = (arg_val >> 8) & 0xFF; // High Byte (MSB)
      packet[3] = arg_val & 0xFF;        // Low Byte (LSB)
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK;
    }
  }
  return returnVal;
}

// =================================================================
// "CALLBACK" FUNCTIONS (Called by Response Handler)
// =================================================================

/**
 * @brief Called by parsePulsesResponse when state is SET_HOME.
 * Sets the current *pulse count* as the new absolute home offset.
 */
void executeSetHome() {
  absoluteHomeOffset_pulses = currentAbsolutePosition_pulses;
  positionIsKnown = true;
  Serial.println(">>> Action: Software Home Set OK!");
  Serial.println(">>> New Absolute Home Offset: " + String(absoluteHomeOffset_pulses) + " pulses.");
}

/**
 * @brief Called by parsePulsesResponse when state is GO_HOME.
 * Calculates and executes the relative move to return to home.
 */
void executeGoHome() {
  // Calculate the relative move needed: (Target) - (Current)
  // All units are in PULSES, so no conversion is needed.
  int32_t relativeMove = -(absoluteHomeOffset_pulses - currentAbsolutePosition_pulses);

  Serial.println(">>> Action: Go Home!");
  Serial.println(">>>  Target Home (pulses): " + String(absoluteHomeOffset_pulses));
  Serial.println(">>>  Current Pos (pulses): " + String(currentAbsolutePosition_pulses));
  Serial.println(">>>  Calculated Relative Move: " + String(relativeMove) + " pulses.");
  
  if (relativeMove == 0) {
    Serial.println(">>> Already at home position.");
    return;
  }
  
  sendMovePacket(relativeMove, pendingGoHomeSpeed);
}


// =================================================================
// AUXILIARY FUNCTIONS (Communication & UI)
// =================================================================

/**
 * @brief Helper function to request a PULSE COUNT update.
 * This is the trigger for our state machine.
 */
void triggerPulsesRead() {
  byte packet[] = {DRIVER_ADDR, 0x33}; // 0x33 = read_pulses
  sendPacketWithChecksum(packet, sizeof(packet));
}

/**
 * @brief Central function to send a move (0xFD) packet.
 * @param pulses The number of relative pulses to move (can be negative).
 * @param speed The speed (0-127).
 */
void sendMovePacket(long pulses, int speed) {
  byte packet[7]; 
  packet[0] = DRIVER_ADDR;
  packet[1] = 0xFD; // 'Run by serial' command
  
  byte direction = (pulses < 0) ? 1 : 0;
  pulses = abs(pulses);
  byte speed_val = (byte)abs(speed) & 0x7F;
  packet[2] = (direction << 7) | speed_val; // VAL byte

  // Split 'long' pulses into 4 bytes (MSB first)
  packet[3] = (pulses >> 24) & 0xFF; 
  packet[4] = (pulses >> 16) & 0xFF;
  packet[5] = (pulses >> 8) & 0xFF;
  packet[6] = pulses & 0xFF;         
  
  sendPacketWithChecksum(packet, sizeof(packet));
}

/**
 * @brief Sends a data packet to the driver, calculating and appending the checksum.
 * @param packetData The byte array of the command (e.g., {0xE0, 0xF3, 0x01}).
 * @param dataLength The total number of bytes in the packetData array.
 */
void sendPacketWithChecksum(byte* packetData, int dataLength) {
  byte checksum = 0;
  
  Serial.print("Sending Packet:   [ ");
  for (int i = 0; i < dataLength; i++) {
    checksum += packetData[i];
    
    Serial.print("0x");
    if (packetData[i] < 0x10) Serial.print("0");
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

// =================================================================
// RESPONSE PARSING LOGIC
// =================================================================

/**
 * @brief Handles multi-byte responses from the driver (non-blocking).
 * Collects bytes from Serial2 into rxBuffer until a 'messageTimeout'
 * of silence occurs, then routes to the main parser.
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
    parseAndPrintResponse(rxBuffer, rxBufferIndex);
    rxBufferIndex = 0; // Reset for next message
  }
}

/**
 * @brief Validates the checksum of a received buffer.
 * @param buffer The byte array of the response.
 * @param length The *total length* of the packet, including the checksum byte.
 * @return bool 'true' if the checksum is valid, 'false' if not.
 */
bool validateChecksum(byte* buffer, int length) {
  byte checksum = 0;
  // Sum all bytes *except* the last one
  for (int i = 0; i < length - 1; i++) {
    checksum += buffer[i];
  }
  // Compare calculated sum to the *last* byte
  return (checksum == buffer[length - 1]);
}

/**
 * @brief Prints a standard checksum error message.
 * @param buffer The corrupted packet buffer.
 * @param length The length of the corrupted packet.
 */
void printChecksumError(byte* buffer, int length) {
    Serial.print("Driver Response -> [ CHECKSUM ERROR! Data: ");
    for (int i = 0; i < length; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println("]");
}

/**
 * @brief Prints any unrecognized packet in raw hex format.
 * @param message A custom message (e.g., "Unknown packet format").
 * @param buffer The packet buffer.
 * @param length The length of the packet.
 */
void printRawHex(const char* message, byte* buffer, int length) {
    Serial.print("Driver Response (" + String(message) + ") <- [ ");
    for (int i = 0; i < length; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println("]");
}

/**
 * @brief Main router for parsing responses from the driver.
 * Includes hotfixes for known firmware bugs (e.g., extra null bytes).
 * @param buffer The buffer of bytes from the response.
 * @param length The length of the buffer.
 */
void parseAndPrintResponse(byte* buffer, int length) {

  // --- Hotfix: Driver sends 4 bytes for a 3-byte Status packet ---
  // e.g., [E0 01 E1 00]
  if (length == 4 && buffer[0] == DRIVER_ADDR && buffer[3] == 0x00) {
    if (validateChecksum(buffer, 3)) { // Validate first 3 bytes
      parseStatusResponse(buffer); // Parse first 3 bytes
      return;
    } else {
      printChecksumError(buffer, length);
      return;
    }
  }

  // --- Hotfix: Driver sends 9 bytes for an 8-byte Encoder packet ---
  // e.g., [E0 ... D8 00]
  if (length == 9 && buffer[0] == DRIVER_ADDR && buffer[8] == 0x00) {
    if (validateChecksum(buffer, 8)) { // Validate first 8 bytes
      parseEncoderResponse(buffer); // Parse first 8 bytes
      return;
    } else {
      printChecksumError(buffer, length);
      return;
    }
  }
  
  // --- Hotfix: Driver sends 7 bytes for a 6-byte Pulses packet ---
  // e.g., [E0 ... 83 00]
  else if (length == 7 && buffer[0] == DRIVER_ADDR && buffer[6] == 0x00) {
    if (validateChecksum(buffer, 6)) { // Validate first 6 bytes
      parsePulsesResponse(buffer); // Parse first 6 bytes
      return;
    } else{
      printChecksumError(buffer, length);
      return;
    }
  }

  // If no hotfix matched, try to validate the packet as-is
  if (!validateChecksum(buffer, length)) {
    printChecksumError(buffer, length);
    return;
  }

  // Standard (non-buggy) packet handling
  switch (length) {
    case 3:
      parseStatusResponse(buffer);
      break;
    case 4:
      parseAngleErrorResponse(buffer);
      break;
    case 6:
      // This is the correct packet for position tracking
      parsePulsesResponse(buffer); // <-- STATE MACHINE CORE
      break;
    case 8:
      // This is the 'read_encoder' response
      parseEncoderResponse(buffer);
      break;
    default:
      printRawHex("Unknown packet format", buffer, length);
  }
}

/**
 * @brief Parses and prints a 3-byte Status response.
 * Format: [0xE0] [Status] [Checksum]
 * @param buffer A 3-byte buffer.
 */
void parseStatusResponse(byte* buffer) {
  byte status = buffer[1];
  Serial.print("Driver Response -> Status: ");
  switch (status) {
    case 0x00:
      Serial.println("Command FAILED");
      break;
    case 0x01:
      Serial.println("Command OK / Move STARTED");
      break;
    case 0x02:
      Serial.println("Move COMPLETE");
      break;
    default:
      Serial.println("Unknown status code 0x" + String(status, HEX));
  }
}

/**
 * @brief Parses a 6-byte Pulse Count response and triggers the state machine.
 * This is the core of the absolute position logic.
 * Format: [0xE0] [Pulses(4 bytes)] [Checksum]
 * @param buffer A 6-byte buffer.
 */
void parsePulsesResponse(byte* buffer) {
  // 1. Parse the 32-bit absolute pulse count
  // Bytes 1-4 are int32_t 'pulses' (Big-Endian)
  int32_t pulses = (int32_t)(buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4]);
  // Byte 5 is the checksum
  
  currentAbsolutePosition_pulses = pulses;
  positionIsKnown = true;
  
  // 2. Print human-readable output
  Serial.println("Driver Response -> Pulse Count Read:");
  Serial.println("  >>> Absolute Position (pulses): " + String(currentAbsolutePosition_pulses));

  // 3. --- STATE MACHINE LOGIC ---
  // Check if this read was triggered by a command
  switch (currentAction) {
    case SET_HOME:
      executeSetHome();
      break;
    case GO_HOME:
      executeGoHome();
      break;
    case NONE:
      // Manual 'read_pulses' command. Do nothing.
      break;
  }
  
  // 4. Reset the state machine
  currentAction = NONE;
}

/**
 * @brief Parses an 8-byte Encoder response.
 * This is the raw encoder angle, not the pulse count.
 * Format: [0xE0] [Carry(4b)] [Value(2b)] [Checksum]
 * @param buffer An 8-byte buffer.
 */
void parseEncoderResponse(byte* buffer) {
  int32_t carry = (int32_t)(buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4]);
  uint16_t value = (uint16_t)(buffer[5] << 8 | buffer[6]);
  
  int64_t pos = (int64_t)carry * 65536 + (int64_t)value;

  Serial.println("Driver Response -> Raw Encoder Read:");
  Serial.println("  Carry (Revolutions): " + String((long)carry));
  Serial.println("  Value (Position 0-65535): " + String(value));
  Serial.println("  (Note: This is the raw encoder tick value, not pulse count)");
}

/**
 * @brief Parses and prints a 4-byte Angle Error response.
 * Format: [0xE0] [Error(2b)] [Checksum]
 * @param buffer A 4-byte buffer.
 */
void parseAngleErrorResponse(byte* buffer) {
  int16_t error_raw = (int16_t)(buffer[1] << 8 | buffer[2]);
  float error_degrees = (float)error_raw / 182.044; // 65536 / 360 = 182.044
  
  Serial.println("Driver Response -> Angle Error Read:");
  Serial.println("  Raw Error Value: " + String(error_raw));
  Serial.println("  Angle Error (deg): " + String(error_degrees, 3));
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
  Serial.println("set_home               : (Software) Sets current *pulse count* as absolute 0.");
  Serial.println("go_home <speed>        : (Software) Returns to absolute 0 *pulse count*.");
  Serial.println("--- Driver-Internal Homing (Single-Rotation) ---");
  Serial.println("set_zero               : (Driver) Sets the current *angle* as 0 datum.");
  Serial.println("goto_zero              : (Driver) Returns to the 0 *angle* datum.");
  Serial.println("set_zero_mode <0-2>    : Sets driver homing mode (0=Off, 1=Dir, 2=Near)");
  Serial.println("set_zero_speed <0-4>   : Sets driver homing speed (0=fast, 4=slow)");
  Serial.println("set_zero_dir <0|1>     : Sets driver homing direction (0=CW, 1=CCW)");
}

/**
 * @brief Prints help for Category 3.
 */
void printHelp_Feedback() {
  Serial.println("--- 3. Read (Feedback) Commands ---");
  Serial.println("read_pulses            : Reads absolute multi-rotation *pulse count* (int32)");
  Serial.println("read_encoder           : Reads absolute multi-rotation *raw encoder ticks* (int64)");
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