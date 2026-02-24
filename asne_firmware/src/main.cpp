#include <Arduino.h>
#include <ESP32Servo.h>

// SERVO
#define SERVO_PIN 16

// SBUS
#define SBUS_PIN 4
#define SBUS_BAUD 100000
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00

HardwareSerial sbus_serial(1); // UART1

uint8_t sbus_data[25];

uint16_t raw_channels[16];
float normalized_channels[16];

bool failsafe = false;
bool frame_lost = false;

enum Channel_idx
{
  Joy_R_LR = 0,
  Joy_R_UD,
  Joy_L_UD,
  Joy_L_LR,
  SW_B,
  Vr_B,
  SW_A,
  Vr_A,
  CH9,
  CH10,
  CH11,
  CH12,
  CH13,
  CH14,
  CH15,
  CH16,
};

const char *CHANNEL_LABELS[16] = {
    "Joy_R_LR",
    "Joy_R_UD",
    "Joy_L_UD",
    "Joy_L_LR",
    "SW_B",
    "Vr_B",
    "SW_A",
    "Vr_A",
    "CH9",
    "CH10",
    "CH11",
    "CH12",
    "CH13",
    "CH14",
    "CH15",
    "CH16",
};

#define JOY_DEADZONE 0.1

#define JOY_R_LR_CENTER 967
#define JOY_R_UD_CENTER 988
#define JOY_L_UD_CENTER 987
#define JOY_L_LR_CENTER 1000
#define JOY_AXIS_SCALE 800
#define VR_MIN 200
#define VR_MAX 1800

void parse_sbus(uint8_t *data)
{
  raw_channels[0] = ((data[1] | data[2] << 8) & 0x07FF);
  raw_channels[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF);
  raw_channels[2] = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
  raw_channels[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF);
  raw_channels[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF);
  raw_channels[5] = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
  raw_channels[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
  raw_channels[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
  raw_channels[8] = ((data[12] | data[13] << 8) & 0x07FF);
  raw_channels[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF);
  raw_channels[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF);
  raw_channels[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF);
  raw_channels[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF);
  raw_channels[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF);
  raw_channels[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF);
  raw_channels[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF);

  frame_lost = data[23] & (1 << 2);
  failsafe = data[23] & (1 << 3);
}

void normalize_channels()
{
  // JOYSTICK -> [-1, 1]
  normalized_channels[0] = float(raw_channels[0] - JOY_R_LR_CENTER) / JOY_AXIS_SCALE;
  normalized_channels[1] = float(raw_channels[1] - JOY_R_UD_CENTER) / -JOY_AXIS_SCALE;
  normalized_channels[2] = float(raw_channels[2] - JOY_L_UD_CENTER) / JOY_AXIS_SCALE;
  normalized_channels[3] = float(raw_channels[3] - JOY_L_LR_CENTER) / JOY_AXIS_SCALE;

  for (int i = 0; i < 4; i++)
  {
    normalized_channels[i] = constrain(normalized_channels[i], -1.0f, 1.0f);

    if (abs(normalized_channels[i]) < JOY_DEADZONE)
    {
      normalized_channels[i] = 0.0;
    }
  }

  // SWITCHES
  normalized_channels[4] = (raw_channels[4] == 200) ? 1 : (raw_channels[4] == 1000) ? 0
                                                                                    : -1;
  normalized_channels[6] = (raw_channels[6] == 200) ? 1 : (raw_channels[6] == 1800) ? -1
                                                                                    : 0;

  // KNOBS
  normalized_channels[5] = 1 - float(raw_channels[5] - VR_MIN) / (VR_MAX - VR_MIN);
  normalized_channels[7] = float(raw_channels[7] - VR_MIN) / (VR_MAX - VR_MIN);

  // REST OF CHANNELS (not used by reciever = no normalization)
  for (int i = 8; i < 16; i++)
    normalized_channels[i] = float(raw_channels[i]);
}

// FOR DEBUGGING
void log_channels()
{
  for (int i = 0; i < 16; i++)
  {
    Serial.print(CHANNEL_LABELS[i]);
    Serial.print(": ");
    Serial.println(normalized_channels[i]);
  }
}

bool read_sbus()
{
  if (!sbus_serial.available())
    return false;

  if (sbus_serial.read() != SBUS_START_BYTE)
    return false;

  sbus_data[0] = SBUS_START_BYTE;
  if (sbus_serial.readBytes(&sbus_data[1], 24) < 24)
    return false;

  if (sbus_data[24] != SBUS_END_BYTE)
  {
    while (sbus_serial.available())
      sbus_serial.read();
    return false;
  }

  parse_sbus(sbus_data);
  normalize_channels();
  return true;
}

void print_channels()
{
  for (int i = 0; i < 16; i++)
  {
    Serial.print(normalized_channels[i], 4);
    Serial.print(", ");
  }
  Serial.print(failsafe);
  Serial.print(", ");
  Serial.println(frame_lost);
}

void setup()
{
  Serial.begin(115200); // USB -> Jetson
  delay(1000);

  sbus_serial.begin(SBUS_BAUD, SERIAL_8E2, SBUS_PIN);
}

void loop()
{
  // SBUS
  if (read_sbus())  
  {
    print_channels();
  }
}