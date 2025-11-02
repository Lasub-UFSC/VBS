/**
 * @file MKS_SERVO42C_UART_Bridge.ino
 * @version 1.4.4 (Smart Response Parser)
 * @author Lucas Barcaro (comments AI generated, if any error, please contact the author)
 * @date October 2025
 *
 * @brief This version adds a smart parser for driver responses.
 * Instead of printing raw hex bytes, it validates the checksum,
 * determines the message type based on its length, and prints a
 * human-readable interpretation.
 *
 * @features
 * - Human-readable command parser.
 * - Automatic checksum calculation (sending).
 * - Automatic checksum validation (receiving).
 * - Smart response parser (e.g., "Command OK", "Encoder Value: 16384").
 * - Data-driven routing for commands.
 * - Interactive 'help' menu.
 */

// --- UART2 (Driver) Settings ---
#define DRIVER_BAUD_RATE 38400
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17

// --- Receive Buffer Settings ---
#define RX_BUFFER_SIZE 64
byte rxBuffer[RX_BUFFER_SIZE];
int rxBufferIndex                 = 0;
unsigned long lastByteTime        = 0;
const long messageTimeout         = 20;

// --- Driver Protocol Constants ---
const byte DRIVER_ADDR            = 0xE0;
const byte MAX_SPEED_VALUE        = 0x7F;
const int MAX_TORQUE_VALUE        = 0x0480;

// =================================================================
// STATUS CODES
// =================================================================
const int8_t STATUS_OK             =  0;
const int8_t ERR_UNKNOWN_COMMAND   = -1;
const int8_t ERR_INVALID_ARGS      = -2;
const int8_t ERR_ARG_OUT_OF_RANGE  = -3;

// =================================================================
// COMMAND DICTIONARIES (Data-Driven Routing)
// =================================================================
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

struct SingleByteArgCommand {
  const char* name;
  byte functionCode;
  int minVal;
  int maxVal;
  const char* errorMsg;
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

struct TwoByteArgCommand {
  const char* name;
  byte functionCode;
  long minVal;
  long maxVal;
  const char* errorMsg;
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
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial2.begin(DRIVER_BAUD_RATE, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  Serial.println("\n--- MKS Controller (v4.4 - Smart Parser) ---");
  Serial.println("Type 'help' for a list of commands.");
}


void loop() {
  if (Serial.available()) {
    String rawInput = Serial.readStringUntil('\n');
    rawInput.trim();
    if (rawInput.length() > 0) {
      Serial.print("\n> ");
      Serial.println(rawInput);
      parseAndSendCommand(rawInput);
    }
  }
  // handleDriverResponses() agora fará o parsing inteligente
  handleDriverResponses();
}

// =================================================================
// MASTER COMMAND ROUTER
// =================================================================
void parseAndSendCommand(String line) {
  line.toLowerCase();
  String command = "";
  String args = "";
  int firstSpace = line.indexOf(' ');

  if (firstSpace == -1) {
    command = line;
  } else {
    command = line.substring(0, firstSpace);
    args = line.substring(firstSpace + 1);
    args.trim();
  }

  int8_t returnVal = ERR_UNKNOWN_COMMAND;

  if (command == "help") {
    returnVal = handleHelpCommand(args);
  } 
  else if (command == "move") {
    returnVal = handleMoveCommand(args);
  } 
  else if (command == "spin") {
    returnVal = handleSpinCommand(args);
  } 
  else {
    returnVal = findAndSendSingleByteArgCommand(command, args);
    if (returnVal == ERR_UNKNOWN_COMMAND) {
      returnVal = findAndSendTwoByteArgCommand(command, args);
    }
    if (returnVal == ERR_UNKNOWN_COMMAND) {
      returnVal = findAndSendSimpleCommand(command);
    }
  }

  if (returnVal == ERR_UNKNOWN_COMMAND) {
    Serial.println("Error: Command '" + command + "' not recognized.");
  }
}

// =================================================================
// COMMAND DICTIONARY "FINDER" FUNCTIONS (Routing Logic)
// =================================================================
int8_t findAndSendSingleByteArgCommand(String command, String args) {
  int8_t returnVal = ERR_UNKNOWN_COMMAND;
  for (int i = 0; i < numSingleByteCommands; i++) {
    if (command == singleByteCommands[i].name) {
      returnVal = handleSingleByteArgCommand(
        args,
        singleByteCommands[i].functionCode,
        singleByteCommands[i].minVal,
        singleByteCommands[i].maxVal,
        singleByteCommands[i].errorMsg
      );
      break; 
    }
  }
  return returnVal;
}


int8_t findAndSendTwoByteArgCommand(String command, String args) {
  int8_t returnVal = ERR_UNKNOWN_COMMAND;
  for (int i = 0; i < numTwoByteCommands; i++) {
    if (command == twoByteCommands[i].name) {
      returnVal = handleTwoByteArgCommand(
        args,
        twoByteCommands[i].functionCode,
        twoByteCommands[i].minVal,
        twoByteCommands[i].maxVal,
        twoByteCommands[i].errorMsg
      );
      break;
    }
  }
  return returnVal;
}


int8_t findAndSendSimpleCommand(String command) {
  int8_t returnVal = ERR_UNKNOWN_COMMAND; 
  for (int i = 0; i < numSimpleCommands; i++) {
    if (command == simpleCommands[i].name) {
      int cmdLength = simpleCommands[i].length;
      byte packet[1 + cmdLength]; 
      packet[0] = DRIVER_ADDR;
      memcpy(&packet[1], simpleCommands[i].bytes, cmdLength);
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK; 
      break; 
    }
  }
  return returnVal;
}

// =================================================================
// COMMAND LOGIC HANDLERS (Unchanged from v4.3)
// =================================================================
int8_t handleHelpCommand(String args) {
    int8_t returnVal = STATUS_OK;
    if (args.length() == 0) {
        printHelp_Main();
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


int8_t handleMoveCommand(String args) {
  int8_t returnVal = ERR_INVALID_ARGS; 
  int spaceIndex = args.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("Error: 'move' requires <pulses> and <speed> (e.g., move 3200 30)");
  } else {
    long pulses = args.substring(0, spaceIndex).toInt();
    int speed_raw = args.substring(spaceIndex + 1).toInt();
    byte speed_val = (byte)abs(speed_raw);
    if (speed_val > MAX_SPEED_VALUE) {
      Serial.println("Error: Speed must be between -" + String(MAX_SPEED_VALUE) + " and " + String(MAX_SPEED_VALUE));
      returnVal = ERR_ARG_OUT_OF_RANGE;
    } else {
      byte packet[7]; 
      packet[0] = DRIVER_ADDR;
      packet[1] = 0xFD; 
      byte direction = (pulses < 0) ? 1 : 0;
      pulses = abs(pulses);
      packet[2] = (direction << 7) | speed_val;
      packet[3] = (pulses >> 24) & 0xFF;
      packet[4] = (pulses >> 16) & 0xFF;
      packet[5] = (pulses >> 8) & 0xFF;
      packet[6] = pulses & 0xFF;         
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK; 
    }
  }
  return returnVal;
}


int8_t handleSpinCommand(String args) {
  int8_t returnVal = ERR_INVALID_ARGS;
  int speed_raw = args.toInt();
  byte speed_val = (byte)abs(speed_raw);
  if (speed_raw == 0) {
    Serial.println("Error: 'spin' requires a non-zero speed (e.g., spin 10)");
  } 
  else if (speed_val > MAX_SPEED_VALUE) {
    Serial.println("Error: Speed must be between -" + String(MAX_SPEED_VALUE) + " and " + String(MAX_SPEED_VALUE));
    returnVal = ERR_ARG_OUT_OF_RANGE;
  } 
  else {
    byte direction = (speed_raw < 0) ? 1 : 0;
    byte val = (direction << 7) | speed_val; 
    byte packet[] = {DRIVER_ADDR, 0xF6, val};
    sendPacketWithChecksum(packet, sizeof(packet));
    returnVal = STATUS_OK;
  }
  return returnVal;
}


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
      packet[2] = (arg_val >> 8) & 0xFF; 
      packet[3] = arg_val & 0xFF;        
      sendPacketWithChecksum(packet, sizeof(packet));
      returnVal = STATUS_OK;
    }
  }
  return returnVal;
}


// =================================================================
// AUXILIARY FUNCTIONS (Communication & UI)
// =================================================================
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
  Serial.print("0x");
  if (checksum < 0x10) Serial.print("0");
  Serial.print(checksum, HEX);
  Serial.println(" ] (Checksum)");
  Serial2.write(packetData, dataLength);
  Serial2.write(checksum);
}


// =================================================================
// RESPONSE PARSING LOGIC (New)
// =================================================================
void handleDriverResponses() {
  while (Serial2.available()) {
    if (rxBufferIndex < RX_BUFFER_SIZE) {
      rxBuffer[rxBufferIndex] = Serial2.read();
      rxBufferIndex++;
      lastByteTime = millis();
    } else {
      Serial2.read();
      lastByteTime = millis();
    }
  }

  if (rxBufferIndex > 0 && (millis() - lastByteTime > messageTimeout)) {
    parseAndPrintResponse(rxBuffer, rxBufferIndex);
    rxBufferIndex = 0;
  }
}


bool validateChecksum(byte* buffer, int length) {
  byte checksum = 0;
  for (int i = 0; i < length - 1; i++) {
    checksum += buffer[i];
  }
  return (checksum == buffer[length - 1]);
}


void parseAndPrintResponse(byte* buffer, int length) {
  if (!validateChecksum(buffer, length)) {
    Serial.print("Driver Response -> [ CHECKSUM ERROR! Data: ");
    for (int i = 0; i < length; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println("]");
  }

  switch (length) {
    case 3:
      parseStatusResponse(buffer);
      break;
    case 4:
      parseAngleErrorResponse(buffer);
      break;
    case 8:
      parseEncoderResponse(buffer);
      break;
    default:
      Serial.print("Driver Response (Unknown Format) <- [ ");
      for (int i = 0; i < length; i++) {
        Serial.print("0x");
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println("]");
  }
}


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


void parseEncoderResponse(byte* buffer) {
  
  int32_t carry = (int32_t)(buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4]);
  uint16_t value = (uint16_t)(buffer[5] << 8 | buffer[6]);
  
  Serial.println("Driver Response -> Encoder Read:");
  Serial.println("  Carry (Revolutions): " + String(carry));
  Serial.println("  Value (Position 0-65535): " + String(value));
}


void parseAngleErrorResponse(byte* buffer) {
  int16_t error = (int16_t)(buffer[1] << 8 | buffer[2]);
  
  float error_degrees = (float)error / 182.044; 
  
  Serial.println("Driver Response -> Angle Error Read:");
  Serial.println("  Raw Error Value: " + String(error));
  Serial.println("  Angle Error (deg): " + String(error_degrees, 3));
}

// =================================================================
// HELP MENU PRINTING FUNCTIONS
// =================================================================
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
void printHelp_Homing() {
  Serial.println("--- 2. Homing (Zero) Commands ---");
  Serial.println("set_zero               : Sets the current position as 0");
  Serial.println("goto_zero              : Moves to the 0 position");
  Serial.println("set_zero_mode <0-2>    : Sets homing mode (0=Off, 1=Dir, 2=Near)");
  Serial.println("set_zero_speed <0-4>   : Sets homing speed (0=fast, 4=slow)");
  Serial.println("set_zero_dir <0|1>     : Sets homing direction (0=CW, 1=CCW)");
}
void printHelp_Feedback() {
  Serial.println("--- 3. Read (Feedback) Commands ---");
  Serial.println("read_encoder           : Reads the current encoder position");
  Serial.println("read_pulses            : Reads the received pulse counter");
  Serial.println("read_error_angle       : Reads the current angle error");
  Serial.println("read_en_status         : Reads the Enable pin status");
  Serial.println("read_protection_state  : Reads protection status (1=locked, 2=ok)");
  Serial.println("release_protection     : Releases motor from a locked-rotor fault");
}
void printHelp_PID() {
  Serial.println("--- 4. Configuration (PID) Commands ---");
  Serial.println("set_kp <0-65535>       : Sets Proportional gain (Kp)");
  Serial.println("set_ki <0-65535>       : Sets Integral gain (Ki)");
  Serial.println("set_kd <0-65535>       : Sets Derivative gain (Kd)");
}
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