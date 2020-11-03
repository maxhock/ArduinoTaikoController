#include "AnalogReadNow.h"
//#include "Kalman.h"

//Kalman xFilter(0.125,32,1023,0);
//Kalman yFilter(0.125,32,1023,0);

#define DEBUG_OUTPUT
//#define DEBUG_OUTPUT_LIVE
//#define DEBUG_TIME
//#define DEBUG_DATA

//#define ENABLE_KEYBOARD
#define ENABLE_NS_JOYSTICK

#define HAS_BUTTONS

// Definitions for Sensors
#define Top_Left 0
#define Top_Right 3
#define Bottom_Left 2
#define Bottom_Right 1
#define Bottom 4
#define Top 5

#ifdef ENABLE_KEYBOARD
#include <Keyboard.h>
#endif

#ifdef ENABLE_NS_JOYSTICK
#include "Joystick.h"
const int led_pin[4] = {8, 9, 10, 11};
const int sensor_button[4] = {SWITCH_BTN_ZL, SWITCH_BTN_B, SWITCH_BTN_LCLICK, SWITCH_BTN_ZR};
#endif

#ifdef HAS_BUTTONS
int button_state[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int button_cd[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#ifdef ENABLE_KEYBOARD
const int button_key[16] = {
    KEY_UP_ARROW, KEY_RIGHT_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW,
    'k', 'j', 'f', 'd',
    KEY_PAGE_DOWN, KEY_PAGE_UP, KEY_ESC, ' ',
    KEY_F1, 'q', 0 /*Fn1*/, 0 /*Fn2*/
};
#endif
#ifdef ENABLE_NS_JOYSTICK
const int button[16] = {
    0 /*SWITCH_HAT_U*/, 0 /*SWITCH_HAT_R*/, 0 /*SWITCH_HAT_D*/, 0 /*SWITCH_HAT_L*/,
    SWITCH_BTN_X, SWITCH_BTN_A, SWITCH_BTN_B, SWITCH_BTN_Y,
    SWITCH_BTN_L, SWITCH_BTN_R, SWITCH_BTN_SELECT, SWITCH_BTN_START,
    SWITCH_BTN_CAPTURE, SWITCH_BTN_HOME, 0 /*Fn1*/, 0 /*Fn2*/
};
const int hat_mapping[16] = {
    SWITCH_HAT_CENTER,
    SWITCH_HAT_U,
    SWITCH_HAT_R,
    SWITCH_HAT_UR,
    SWITCH_HAT_D,
    SWITCH_HAT_CENTER,
    SWITCH_HAT_DR,
    SWITCH_HAT_R,
    SWITCH_HAT_L,
    SWITCH_HAT_UL,
    SWITCH_HAT_CENTER,
    SWITCH_HAT_U,
    SWITCH_HAT_DL,
    SWITCH_HAT_L,
    SWITCH_HAT_D,
    SWITCH_HAT_CENTER,
};
#endif
#endif

const long min_threshold = 62500;
const long cd_length = 10000;
const float k_threshold = 1.5;
const float k_decay = 0.95;

// defining outer and inner radius to eventually replace other method
const long inner_radius = 250;
const long outer_radius = 400;
const int loop_threshold = 50;

const int pin[4] = {A0, A3, A2, A1};
const int key[4] = {'d', 'f', 'j', 'k'};
const float sens[4] = {0.8, 1.0, 1.0, 0.8};

const int key_next[4] = {3, 2, 0, 1};

const long cd_stageselect = 200000;

bool stageselect = false;
bool stageresult = false;

float threshold = 20;
int raw[4] = {0, 0, 0, 0};
float level[4] = {0, 0, 0, 0};
long cd[4] = {0, 0, 0, 0};
bool down[4] = {false, false, false, false};
#ifdef ENABLE_NS_JOYSTICK
uint8_t down_count[4] = {0, 0, 0, 0};
#endif

typedef unsigned long time_t;
time_t t0 = 0;
time_t dt = 0, sdt = 0;

void sample()
{
  int prev[4] = {raw[0], raw[1], raw[2], raw[3]};
  raw[0] = analogRead(pin[0]);
  raw[1] = analogRead(pin[1]);
  raw[2] = analogRead(pin[2]);
  raw[3] = analogRead(pin[3]);
  for (int i = 0; i < 4; ++i)
    level[i] = abs(raw[i] - prev[i]) * sens[i];
}

void sampleSingle(int i)
{
  int prev = raw[i];
  raw[i] = analogReadNow();
  //Squared result to get sharper peaks
  level[i] = (abs(raw[i] - prev) * sens[i]);
  analogSwitchPin(pin[key_next[i]]);
}

void setup()
{
  analogReference(DEFAULT);
  analogSwitchPin(pin[0]);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#ifdef ENABLE_NS_JOYSTICK
  for (int i = 0; i < 8; ++i)
    pinMode(i, INPUT_PULLUP);
  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(led_pin[i], HIGH);
    pinMode(led_pin[i], OUTPUT);
  }
#endif
#ifdef ENABLE_KEYBOARD
  Keyboard.begin();
#endif
  t0 = micros();
  Serial.begin(9600);
}

void parseSerial()
{
  static char command = -1;
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (command == -1)
      command = c;
    else
    {
      switch (command)
      {
      case 'C':
        Serial.write('C');
        Serial.write(c);
        Serial.flush();
        break;
      case 'S':
        stageselect = (c == '1');
        digitalWrite(LED_BUILTIN, stageselect ? HIGH : LOW);
        break;
      case 'R':
        stageresult = (c == '1');
        digitalWrite(LED_BUILTIN, stageresult ? HIGH : LOW);
        break;
      }
      command = -1;
    }
  }
}

void loop()
{
  //loop_test2(); return;

  time_t t1 = micros();
  dt = t1 - t0;
  sdt += dt;
  t0 = t1;

  static float vector_x;
  static float vector_y;
  static float vector_amp;

  bool sens_state[4] = {0, 0, 0, 0};
  static int sens_count[6] = {0, 0, 0, 0, 0, 0};
  int max_count = 0;
  int max_index = -1;
  static int loop_count = 0;

  // add every sensor to xy-coordinates exp. Top_Right is [1,1] Bottom_Right is [1,-1]
  // for (int i = 0; i < 4; i++)
  // {
  //   sampleSingle(i);
  // }
  sample();

  vector_x = level[Top_Right] + level[Bottom_Right] - level[Top_Left] - level[Bottom_Left];
  vector_y = level[Top_Right] + level[Top_Left] - level[Bottom_Right] - level[Bottom_Left];
  vector_amp = sqrt(sq(vector_x) + sq(vector_y));

  if (vector_amp >= inner_radius && vector_amp <= outer_radius)
  {
    loop_count++;

    if (vector_y <= 0)
    {
      if (vector_x <= 0)
      {
        sens_count[Bottom_Left]++;
      }
      else
      {
        sens_count[Bottom_Right]++;
      }
    }
    else
    {
      if (vector_x <= 0)
      {
        sens_count[Top_Left]++;
      }
      else
      {
        sens_count[Top_Right]++;
      }
    }
  }
  else if (vector_amp >= outer_radius)
  {
    loop_count++;

    if (vector_y <= 0)
    {
      sens_count[Bottom]++;
    }
    else
    {
      sens_count[Top]++;
    }
  }
  else if (loop_count >= loop_threshold)
  {
    loop_count = 0;
    for (int i = 0; i < 6; i++)
    {
      if (sens_count[i] > max_count)
      {
        max_count = sens_count[i];
        max_index = i;
      }
    }
    if (max_index == Top)
    {
      sens_state[Top_Left] = true;
      sens_state[Top_Right] = true;
    }
    else if (max_index == Bottom)
    {
      sens_state[Bottom_Left] = true;
      sens_state[Bottom_Right] = true;
    }
    else if (max_index != -1)
    {
      sens_state[max_index] = true;
    }
#ifdef DEBUG_DATA
    if (sens_state[Top_Left] || sens_state[Top_Right] || sens_state[Bottom_Left] || sens_state[Bottom_Right])
    {
      Serial.print("Top_Left: ");
      Serial.print(sens_count[Top_Left]);
      Serial.print("\t");
      Serial.print("Top_Right: ");
      Serial.print(sens_count[Top_Right]);
      Serial.print("\t");
      Serial.print("Top: ");
      Serial.print(sens_count[Top]);
      Serial.print("\t");
      Serial.print("Bottom_Left: ");
      Serial.print(sens_count[Bottom_Left]);
      Serial.print("\t");
      Serial.print("Bottom_Right: ");
      Serial.print(sens_count[Bottom_Right]);
      Serial.print("\t");
      Serial.print("Bottom: ");
      Serial.print(sens_count[Bottom]);
      Serial.println();
    }
#endif
    for (int i = 0; i < 6; i++)
    {
      sens_count[i] = 0;
    }
  }
  else if (sens_count[Top] || sens_count[Top_Left] || sens_count[Top_Right] || sens_count[Bottom] || sens_count[Bottom_Left] || sens_count[Bottom_Right])
  {
    loop_count++;
  }

  for (int i = 0; i < 4; ++i)
  {
    Joystick.Button |= (sens_state[i] ? sensor_button[i] : SWITCH_BTN_NONE);
  }
  Joystick.sendState();
  Joystick.Button = SWITCH_BTN_NONE;

#ifdef DEBUG_OUTPUT_LIVE
  Serial.print("x: ");
  Serial.print(vector_x);
  Serial.print("\t");
  Serial.print("y: ");
  Serial.print(vector_y);
  Serial.print("\t");
  Serial.print("amp: ");
  Serial.print(vector_amp);
  Serial.println();
#endif

#ifdef DEBUG_OUTPUT
  if (sens_state[Top_Left] || sens_state[Top_Right] || sens_state[Bottom_Left] || sens_state[Bottom_Right])
  {
    Serial.print("Top_Left: ");
    Serial.print(sens_state[Top_Left]);
    Serial.print("\t");
    Serial.print("Top_Right: ");
    Serial.print(sens_state[Top_Right]);
    Serial.print("\t");
    Serial.print("Bottom_Left: ");
    Serial.print(sens_state[Bottom_Left]);
    Serial.print("\t");
    Serial.print("Bottom_Right: ");
    Serial.print(sens_state[Bottom_Right]);
    Serial.println();
  }
#endif

  long ddt = 300 - (micros() - t0);
  if (ddt > 3)
    delayMicroseconds(ddt);
}
