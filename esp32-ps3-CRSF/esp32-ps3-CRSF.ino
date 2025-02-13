/*
-------------------------------------------------------------------------
 Simple Arduino trasmisster (Ps3 to CRSF) using ESP32
 this program allow use a PS3 controller to control CRSF tx module
 -------------------------------------------------------------------------
*/

#include <Ps3Controller.h>

// default channel order is AETR, uncomment the line below to change to TAER
// #define TAER

// PRINT INFO, uncomment to enable
#define PRINT_INFO_ON

// CRSF protocol definitions
#define RADIO_ADDRESS 0xEA
#define ADDR_MODULE 0xEE //  Crossfire transmitter
#define TYPE_CHANNELS 0x16
// internal crsf variables
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 991
#define CRSF_CHANNEL_MAX 1811
#define CRSF_TIME_NEEDED_PER_FRAME_US 1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US 4000   // 4 ms 250Hz
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define SERIAL_BAUDRATE 400000
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX 60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE 26
#define CRSF_FRAME_LENGTH 24 // length of type + payload + crc

// UART pins
#define RXD1 12 // UART1 RX default 9
#define TXD1 13 // UART1 TX default 10
#define RXD2 16 // UART2 RX
#define TXD2 17 // UART2 TX

// LED pins
#define LED_CONNECT_PIN 2
#define LED_RGB_PIN 48 // WS2812 of ESP32s3 Super-Mini, not used

// Fixed controller_channels enum
enum controller_channels
{
  stick_L_X,
  stick_L_Y,
  stick_R_X,
  stick_R_Y,
  trigger_L1,
  trigger_R1,
  trigger_L2,
  trigger_R2,
  button_cross,
  button_circle,
  button_square,
  button_triangle,
  button_up,
  button_down,
  button_left,
  button_right,
  button_start,
  button_select,
  button_ps
}; // <-- Added semicolon

// Separate enums for packet rates and transmit power
enum packet_rate
{
  PACKET_RATE_50HZ = 20000,
  PACKET_RATE_100HZ = 10000,
  PACKET_RATE_200HZ = 5000,
  PACKET_RATE_250HZ = 4000,
  PACKET_RATE_500HZ = 2000,
  PACKET_RATE_RACE = 1000,
};

enum transmit_power
{
  POWER_10MW = 0,
  POWER_25MW = 1,
  POWER_50MW = 2,
  POWER_100MW = 3,
  POWER_250MW = 4,
  POWER_500MW = 5,
  POWER_1000MW = 6,
  POWER_2000MW = 7,
};
// CRSF channel packet channel order
#ifdef TAER
enum chan_order
{
  THROTTLE,
  AILERON,
  ELEVATOR,
  RUDDER,
  AUX1,  // (CH5)  ARM switch for Expresslrs
  AUX2,  // (CH6)  angel / airmode change
  AUX3,  // (CH7)  flip after crash
  AUX4,  // (CH8)
  AUX5,  // (CH9)
  AUX6,  // (CH10)
  AUX7,  // (CH11)
  AUX8,  // (CH12)
  AUX9,  // (CH13)
  AUX10, // (CH14)
  AUX11, // (CH15)
  AUX12, // (CH16)
};
#else
enum chan_order
{
  AILERON,
  ELEVATOR,
  THROTTLE,
  RUDDER,
  AUX1,  // (CH5)  ARM switch for Expresslrs
  AUX2,  // (CH6)  angel / airmode change
  AUX3,  // (CH7)  flip after crash
  AUX4,  // (CH8)
  AUX5,  // (CH9)
  AUX6,  // (CH10)
  AUX7,  // (CH11)
  AUX8,  // (CH12)
  AUX9,  // (CH13)
  AUX10, // (CH14)
  AUX11, // (CH15)
  AUX12, // (CH16)
};
#endif

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

int controllerChannels[18]; // Ps3 controller channels
int battery = 0;            // Ps3 controller battery

// global variables for buttons used as self-locking switches
bool stateButtonCross = false;
bool stateButtonCircle = false;
bool stateButtonSquare = false;
bool stateButtonTriangle = false;
bool stateTriggerL1 = false;
bool stateTriggerR1 = false;
bool stateTriggerL2 = false;
bool stateTriggerR2 = false;

void notify()
{
  //---------------------- Battery events ---------------------
  if (battery != Ps3.data.status.battery)
  {
    battery = Ps3.data.status.battery;
    Serial.print("The controller battery is ");
    if (battery == ps3_status_battery_charging)
      Serial.println("charging");
    else if (battery == ps3_status_battery_full)
      Serial.println("FULL");
    else if (battery == ps3_status_battery_high)
      Serial.println("HIGH");
    else if (battery == ps3_status_battery_low)
      Serial.println("LOW");
    else if (battery == ps3_status_battery_dying)
      Serial.println("DYING");
    else if (battery == ps3_status_battery_shutdown)
      Serial.println("SHUTDOWN");
    else
      Serial.println("UNDEFINED");
  }
}

void onConnect()
{
  Serial.println("Connected.");
  digitalWrite(LED_CONNECT_PIN, HIGH);
}
void onDisconnect()
{
  Serial.println("Disconnected.");
  digitalWrite(LED_CONNECT_PIN, LOW);
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(SERIAL_BAUDRATE, SERIAL_8N1, RXD1, TXD1);
  pinMode(LED_CONNECT_PIN, OUTPUT);
  Ps3.begin("20:00:00:00:23:44");
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);

  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) // initialize all CRSF channels to mid
  {
    rcChannels[i] = CRSF_CHANNEL_MID;
  }
  rcChannels[THROTTLE] = CRSF_CHANNEL_MIN; // Throttle to min

  for (uint8_t i = 0; i < 18; i++) // initialize all ps3 channels to 0
  {
    controllerChannels[i] = 0;
  }

  delay(1000);
}

void loop()
{
  uint32_t currentMicros = micros();

  if (Ps3.isConnected())
  {
    readController();
    rcChannels[THROTTLE] = map(controllerChannels[stick_L_Y], 0, 255, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    rcChannels[AILERON] = map(controllerChannels[stick_L_X], 0, 255, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    rcChannels[ELEVATOR] = map(controllerChannels[stick_R_Y], 0, 255, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    rcChannels[RUDDER] = map(controllerChannels[stick_R_X], 0, 255, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    rcChannels[AUX1] = stateTriggerL1 ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX2] = stateTriggerR1 ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX3] = stateTriggerL2 ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX4] = stateTriggerR2 ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX5] = controllerChannels[button_cross] ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX6] = controllerChannels[button_circle] ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX7] = controllerChannels[button_square] ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX8] = controllerChannels[button_triangle] ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX9] = stateButtonCross ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX10] = stateButtonCircle ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX11] = stateButtonSquare ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
    rcChannels[AUX12] = stateButtonTriangle ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
  }
  else
  {
    digitalWrite(LED_CONNECT_PIN, LOW);
    // set fail safe value
    rcChannels[THROTTLE] = CRSF_CHANNEL_MIN; //
    rcChannels[AILERON] = CRSF_CHANNEL_MID;  //
    rcChannels[ELEVATOR] = CRSF_CHANNEL_MID; //
    rcChannels[RUDDER] = CRSF_CHANNEL_MID;   //
    rcChannels[AUX1] = CRSF_CHANNEL_MIN;     // ARM Low
    // loose connection, keep the last values for other channels
  }

  if (currentMicros > crsfTime)
  {
    crsfPreparePacket(crsfPacket, rcChannels);
    Serial1.write(crsfPacket, CRSF_PACKET_SIZE);
    crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
  }
  #if defined(PRINT_INFO_ON)
  if (millis() % 500 == 0){
    Serial.print("Controller Channels: ");
    Serial.printf("stick_LX: %d, stick_LY: %d, stick_RX: %d, stick_RY: %d, ", controllerChannels[stick_L_X], controllerChannels[stick_L_Y], controllerChannels[stick_R_X], controllerChannels[stick_R_Y]);
    Serial.printf("L1: %d, R1: %d, L2: %d, R2: %d, ", controllerChannels[trigger_L1], controllerChannels[trigger_R1], controllerChannels[trigger_L2], controllerChannels[trigger_R2]);
    Serial.printf("Cross: %d, Circle: %d, Square: %d, Triangle: %d, ", controllerChannels[button_cross], controllerChannels[button_circle], controllerChannels[button_square], controllerChannels[button_triangle]);
    Serial.printf("Up: %d, Down: %d, Left: %d, Right: %d, ", controllerChannels[button_up], controllerChannels[button_down], controllerChannels[button_left], controllerChannels[button_right]);
    Serial.printf("Start: %d, Select: %d, PS: %d\n", controllerChannels[button_start], controllerChannels[button_select], controllerChannels[button_ps]);
    Serial.print("RC Channels: ");
    Serial.printf("Throttle: %d, Aileron: %d, Elevator: %d, Rudder: %d, ", rcChannels[THROTTLE], rcChannels[AILERON], rcChannels[ELEVATOR], rcChannels[RUDDER]);
    Serial.printf("Aux1: %d, Aux2: %d, Aux3: %d, Aux4: %d, ", rcChannels[AUX1], rcChannels[AUX2], rcChannels[AUX3], rcChannels[AUX4]);
    Serial.printf("Aux5: %d, Aux6: %d, Aux7: %d, Aux8: %d, ", rcChannels[AUX5], rcChannels[AUX6], rcChannels[AUX7], rcChannels[AUX8]);
    Serial.printf("Aux9: %d, Aux10: %d, Aux11: %d, Aux12: %d\n", rcChannels[AUX9], rcChannels[AUX10], rcChannels[AUX11], rcChannels[AUX12]);
    Serial.print("CRSF Packet: ");
    for (int i = 0; i < CRSF_PACKET_SIZE; i++)
    {
      Serial.printf("%02X ", crsfPacket[i]);
    }
  } else{
    // Serial.println("===========================================");
  }
  #endif // PRINT_INFO_ON)
}

// crc implementation from CRSF protocol document rev7
static uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    crc = crsf_crc8tab[crc ^ *ptr++];
  }
  return crc;
}

void crsfPreparePacket(uint8_t packet[], int channels[])
{
  // The local variable 'output' and the early crc calculation were not used.
  // Pack channels directly into the packet.

  packet[0] = ADDR_MODULE; // Header
  packet[1] = 24;          // length of type (24) + payload + crc
  packet[2] = TYPE_CHANNELS;

  packet[3] = (uint8_t)(channels[0] & 0x07FF);
  packet[4] = (uint8_t)(((channels[0] & 0x07FF) >> 8) | ((channels[1] & 0x07FF) << 3));
  packet[5] = (uint8_t)(((channels[1] & 0x07FF) >> 5) | ((channels[2] & 0x07FF) << 6));
  packet[6] = (uint8_t)((channels[2] & 0x07FF) >> 2);
  packet[7] = (uint8_t)(((channels[2] & 0x07FF) >> 10) | ((channels[3] & 0x07FF) << 1));
  packet[8] = (uint8_t)(((channels[3] & 0x07FF) >> 7) | ((channels[4] & 0x07FF) << 4));
  packet[9] = (uint8_t)(((channels[4] & 0x07FF) >> 4) | ((channels[5] & 0x07FF) << 7));
  packet[10] = (uint8_t)((channels[5] & 0x07FF) >> 1);
  packet[11] = (uint8_t)(((channels[5] & 0x07FF) >> 9) | ((channels[6] & 0x07FF) << 2));
  packet[12] = (uint8_t)(((channels[6] & 0x07FF) >> 6) | ((channels[7] & 0x07FF) << 5));
  packet[13] = (uint8_t)((channels[7] & 0x07FF) >> 3);
  packet[14] = (uint8_t)(channels[8] & 0x07FF);
  packet[15] = (uint8_t)(((channels[8] & 0x07FF) >> 8) | ((channels[9] & 0x07FF) << 3));
  packet[16] = (uint8_t)(((channels[9] & 0x07FF) >> 5) | ((channels[10] & 0x07FF) << 6));
  packet[17] = (uint8_t)((channels[10] & 0x07FF) >> 2);
  packet[18] = (uint8_t)(((channels[10] & 0x07FF) >> 10) | ((channels[11] & 0x07FF) << 1));
  packet[19] = (uint8_t)(((channels[11] & 0x07FF) >> 7) | ((channels[12] & 0x07FF) << 4));
  packet[20] = (uint8_t)(((channels[12] & 0x07FF) >> 4) | ((channels[13] & 0x07FF) << 7));
  packet[21] = (uint8_t)((channels[13] & 0x07FF) >> 1);
  packet[22] = (uint8_t)(((channels[13] & 0x07FF) >> 9) | ((channels[14] & 0x07FF) << 2));
  packet[23] = (uint8_t)(((channels[14] & 0x07FF) >> 6) | ((channels[15] & 0x07FF) << 5));
  packet[24] = (uint8_t)((channels[15] & 0x07FF) >> 3);

  // Compute and append CRC at the end of the packet.
  packet[25] = crsf_crc8(&packet[2], CRSF_PACKET_SIZE - 3); // Note: CRSF_PACKET_SIZE should be defined correctly.
}

void readController()
{
  // Populate controllerChannels with current controller readings.
  controllerChannels[stick_L_X] = Ps3.data.analog.stick.lx;
  controllerChannels[stick_L_Y] = Ps3.data.analog.stick.ly;
  controllerChannels[stick_R_X] = Ps3.data.analog.stick.rx;
  controllerChannels[stick_R_Y] = Ps3.data.analog.stick.ry;
  controllerChannels[trigger_L1] = Ps3.data.button.l1;
  controllerChannels[trigger_R1] = Ps3.data.button.r1;
  controllerChannels[trigger_L2] = Ps3.data.button.l2;
  controllerChannels[trigger_R2] = Ps3.data.button.r2;
  controllerChannels[button_cross] = Ps3.data.button.cross;
  controllerChannels[button_circle] = Ps3.data.button.circle;
  controllerChannels[button_square] = Ps3.data.button.square;
  controllerChannels[button_triangle] = Ps3.data.button.triangle;
  controllerChannels[button_up] = Ps3.data.button.up;
  controllerChannels[button_down] = Ps3.data.button.down;
  controllerChannels[button_left] = Ps3.data.button.left;
  controllerChannels[button_right] = Ps3.data.button.right;
  controllerChannels[button_start] = Ps3.data.button.start;
  controllerChannels[button_select] = Ps3.data.button.select;
  controllerChannels[button_ps] = Ps3.data.button.ps;

  // Use static variables to detect rising edges for toggling.
  static bool prevCross = false, prevCircle = false, prevSquare = false, prevTriangle = false;
  static bool prevL1 = false, prevR1 = false, prevL2 = false, prevR2 = false;

  // Toggle on rising edge (button pressed now, but not in the previous cycle)
  if (Ps3.data.button.cross && !prevCross)
    stateButtonCross = !stateButtonCross;
  if (Ps3.data.button.circle && !prevCircle)
    stateButtonCircle = !stateButtonCircle;
  if (Ps3.data.button.square && !prevSquare)
    stateButtonSquare = !stateButtonSquare;
  if (Ps3.data.button.triangle && !prevTriangle)
    stateButtonTriangle = !stateButtonTriangle;
  if (Ps3.data.button.l1 && !prevL1)
    stateTriggerL1 = !stateTriggerL1;
  if (Ps3.data.button.r1 && !prevR1)
    stateTriggerR1 = !stateTriggerR1;
  if (Ps3.data.button.l2 && !prevL2)
    stateTriggerL2 = !stateTriggerL2;
  if (Ps3.data.button.r2 && !prevR2)
    stateTriggerR2 = !stateTriggerR2;

  // Update previous button states for next cycle.
  prevCross = Ps3.data.button.cross;
  prevCircle = Ps3.data.button.circle;
  prevSquare = Ps3.data.button.square;
  prevTriangle = Ps3.data.button.triangle;
  prevL1 = Ps3.data.button.l1;
  prevR1 = Ps3.data.button.r1;
  prevL2 = Ps3.data.button.l2;
  prevR2 = Ps3.data.button.r2;
}
