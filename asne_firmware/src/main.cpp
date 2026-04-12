#include <Arduino.h>
#include <ESP32Servo.h>

// SERVO
#define SERVO_PIN 5

// SBUS
#define SBUS_PIN 4
#define SBUS_BAUD 100000
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00

uint8_t sbus_data[25];
uint16_t raw_channels[16];
float normalized_channels[16];

bool failsafe = false;
bool frame_lost = false;

Servo myServo;
String inputBuffer = "";

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
  // JOYSTICKS -> [-1, 1]
  normalized_channels[0] = float(raw_channels[0] - JOY_R_LR_CENTER) / JOY_AXIS_SCALE;
  normalized_channels[1] = float(raw_channels[1] - JOY_R_UD_CENTER) / -JOY_AXIS_SCALE;
  normalized_channels[2] = float(raw_channels[2] - JOY_L_UD_CENTER) / JOY_AXIS_SCALE;
  normalized_channels[3] = float(raw_channels[3] - JOY_L_LR_CENTER) / JOY_AXIS_SCALE;

  for (int i = 0; i < 4; i++)
  {
    normalized_channels[i] = constrain(normalized_channels[i], -1.0f, 1.0f);
    if (abs(normalized_channels[i]) < JOY_DEADZONE)
      normalized_channels[i] = 0.0;
  }

  // SWITCHES
  normalized_channels[4] = (raw_channels[4] == 200) ? 1 : (raw_channels[4] == 1000) ? 0
                                                                                    : -1;
  normalized_channels[6] = (raw_channels[6] == 200) ? 1 : (raw_channels[6] == 1800) ? -1
                                                                                    : 0;

  // KNOBS
  normalized_channels[5] = 1 - float(raw_channels[5] - VR_MIN) / (VR_MAX - VR_MIN);
  normalized_channels[7] = float(raw_channels[7] - VR_MIN) / (VR_MAX - VR_MIN);

  // UNUSED CHANNELS
  for (int i = 8; i < 16; i++)
    normalized_channels[i] = float(raw_channels[i]);
}

bool read_sbus()
{
  while (Serial1.available())
  {
    if (Serial1.peek() != SBUS_START_BYTE)
    {
      Serial1.read();
      continue;
    }

    if (Serial1.available() < 25)
    {
      return false;
    }

    Serial1.read();
    sbus_data[0] = SBUS_START_BYTE;

    if (Serial1.readBytes(&sbus_data[1], 24) < 24)
      return false;

    if (sbus_data[24] != SBUS_END_BYTE)
    {
      while (Serial1.available())
        Serial1.read();
      return false;
    }

    parse_sbus(sbus_data);
    normalize_channels();
    return true;
  }
  return false;
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

void read_jetson()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      inputBuffer.trim();
      if (inputBuffer.startsWith("SA:"))
      {
        float angle = inputBuffer.substring(3).toFloat();
        angle = constrain(angle, 0.0, 270.0);
        // Serial.print("Servo angle: ");
        // Serial.println(angle);

        int pulse = (int)map((long)angle, 0, 270, 500, 2500);
        myServo.writeMicroseconds(pulse);
      }
      inputBuffer = "";
    }
    else
    {
      inputBuffer += c;
    }
  }
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

void setup()
{
  Serial.begin(115200);
  delay(2000);
  // Serial.println("Serial OK");

  myServo.attach(SERVO_PIN, 500, 2500);
  // Serial.println("Attached");

  myServo.setPeriodHertz(333);
  // Serial.println("Period set");

  Serial1.begin(SBUS_BAUD, SERIAL_8E2, SBUS_PIN, -1, false);
  // Serial.println("SBUS ready");

  inputBuffer.reserve(64);
  // Serial.println("FULLY INITIALIZED");
}

void loop()
{
  if (read_sbus())
  {
    print_channels();
    // log_channels();
  }

  read_jetson();
  // delay(2000);
  // myServo.writeMicroseconds(500);
  // delay(2000);
  // myServo.writeMicroseconds(2500);
}