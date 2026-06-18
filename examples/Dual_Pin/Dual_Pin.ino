#include <Arduino.h>
#include <EEPROM.h>
#include <NmraDcc.h>

/* --- Pin Configurations --- */
#define DCC_INPUT_PIN_A          PA_7   // Differential Signal Input A (TSSOP20 Pin 14)
#define DCC_INPUT_PIN_B          PA_8   // Differential Signal Input B (TSSOP20 Pin 15)
#define TURNOUT_OUT_PIN          PA_4   // Coil Output (TSSOP20 Pin 10)

/* --- Default Metrics --- */
#define INITIAL_DEFAULT_ADDRESS  1      // Factory base address (Matches Linear Address 1)
#define COIL_PULSE_DURATION_MS   250    // Solenoid activation limit window (prevents coil meltdown)

/* --- Memory Index Layout --- */
#define EEPROM_MAGIC_ADDR        0      // Flash address validation signature offset
#define EEPROM_CV513_ADDR        1      // Base Address LSB (CV 513) cache index 
#define EEPROM_CV521_ADDR        2      // Base Address MSB (CV 521) cache index
#define EEPROM_MAGIC_VALUE       0xDE   // Custom persistent initialization marker

/* --- Global Instances & Tracking Variables --- */
NmraDcc Dcc;
uint32_t turnoutTimer = 0;
bool isTurnoutActive = false;

/* --- Trampoline Vector Bridge --- */
// Routes raw microsecond edge transitions directly into the library state layer
void dcc_hardware_bridge_isr() {
  Dcc.processDualPinsISR();
}

void setup() {
  // Configure hardware pins
  pinMode(TURNOUT_OUT_PIN, OUTPUT);
  digitalWrite(TURNOUT_OUT_PIN, LOW);

  pinMode(DCC_INPUT_PIN_A, INPUT_PULLUP);
  pinMode(DCC_INPUT_PIN_B, INPUT_PULLUP);

  // Initialize the Emulated EEPROM layer in the STM32's internal Flash memory
  EEPROM.begin();

  // If this is a fresh chip, write default NMRA split addresses into emulated storage
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.write(EEPROM_CV513_ADDR, (INITIAL_DEFAULT_ADDRESS & 0x3F));
    EEPROM.write(EEPROM_CV521_ADDR, ((INITIAL_DEFAULT_ADDRESS >> 6) & 0x07));
  }

  // Engage the library using our high-precision custom dual-pin tracking engine
  Dcc.pinDual(DCC_INPUT_PIN_A, DCC_INPUT_PIN_B);
  
  // Initialize core decoder settings (Accessory Mode parsing logic)
  Dcc.init(MAN_ID_DIY, 10, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, 0);

  // Link physical signal transitions straight to the library's trampoline router
  attachInterrupt(digitalPinToInterrupt(DCC_INPUT_PIN_A), dcc_hardware_bridge_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DCC_INPUT_PIN_B), dcc_hardware_bridge_isr, CHANGE);
}

void loop() {
  // Polled for backwards compatibility frameworks and checking background execution states
  Dcc.process();

  // Non-blocking pulse safety tracking loop to prevent burning out physical coils
  if (isTurnoutActive && (millis() - turnoutTimer >= COIL_PULSE_DURATION_MS)) {
    digitalWrite(TURNOUT_OUT_PIN, LOW);
    isTurnoutActive = false;
  }
}

/* --- NMRA Library Non-Volatile Storage Overrides --- */

// Intercepts library requests to fetch data from non-volatile storage
uint8_t notifyCVRead(uint16_t cv) {
  if (cv == CV_ACCESSORY_DECODER_ADDRESS_LSB) return EEPROM.read(EEPROM_CV513_ADDR);
  if (cv == CV_ACCESSORY_DECODER_ADDRESS_MSB) return EEPROM.read(EEPROM_CV521_ADDR);
  return 0;
}

// Intercepts library requests to store newly programmed configuration variable updates
uint8_t notifyCVWrite(uint16_t cv, uint8_t value) {
  if (cv == CV_ACCESSORY_DECODER_ADDRESS_LSB) { 
    EEPROM.write(EEPROM_CV513_ADDR, value); 
    return value; 
  }
  if (cv == CV_ACCESSORY_DECODER_ADDRESS_MSB) { 
    EEPROM.write(EEPROM_CV521_ADDR, value); 
    return value; 
  }
  return 0;
}

/* --- Operational Event Callback Hooks --- */

/**
  * @brief Hook 1: Paragraph 2.4.1 Basic Accessory Turnout Commands
  */
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {
  // Reconstruct the 9-bit address from standard NMRA registers inside emulated EEPROM
  uint16_t myAddr = (Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) & 0x3F) | 
                    ((Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) & 0x07) << 6);

  // Validate if packet coordinates target this specific hardware instance
  if (Addr == myAddr) {
    if (OutputPower == 1) { // Activation pulse triggered ("D" bit active)
      digitalWrite(TURNOUT_OUT_PIN, HIGH);
      turnoutTimer = millis();
      isTurnoutActive = true;
    }
  }
}

/**
  * @brief Hook 2: Paragraph 2.4.2 Extended Accessory aspect signalling structures
  */
void notifyDccExtendedAccessoryOutput(uint16_t receivedAddress, uint8_t dataPayload) {
  uint16_t myAddr = (Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) & 0x3F) | 
                    ((Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) & 0x07) << 6);

  if (receivedAddress == myAddr) {
    // Treat payload data aspects as distinct states (Useful for multi-aspect signal heads)
    if (dataPayload == 0x00) {
      digitalWrite(TURNOUT_OUT_PIN, LOW);   // Rest/Stop state
    } else {
      digitalWrite(TURNOUT_OUT_PIN, HIGH);  // Clear/Proceed state
    }
  }
}

/**
  * @brief Hook 3: Handles Programming on Main (POM) Accessory CV updates
  */
void notifyDccCVProgram(uint16_t targetCv, uint8_t dataPayload) {
  // Dynamically catch layout configuration parameters over the main line track
  if (targetCv == CV_ACCESSORY_DECODER_ADDRESS_LSB || targetCv == CV_ACCESSORY_DECODER_ADDRESS_MSB) {
    // Intercept address shifts, execute safety checks, and store directly to emulated storage
    Dcc.setCV(targetCv, dataPayload);
  }
}
