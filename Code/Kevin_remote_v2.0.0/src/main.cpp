#include <Arduino.h>
#include <configuration.h>
#include <LiquidCrystal.h>
#include <Bounce2.h>
// #include <AltSoftSerial.h>
#include <Wire.h>
#include <remote_control.h>

int t = 0;
int t_button;
int t_lcd;

int xxx;
int yyy;
int packet;
int tune_mode = 0;
int int_store = 0;

bool data_sent = 0;

bool b1 = 0;
bool b2 = 0;
bool b3 = 0;
bool b4 = 0;

bool b1_send = 0;
bool b2_send = 0;
bool b3_send = 0;
bool b4_send = 0;

// foot position vars
struct points x1;
struct points yy1;
struct points z1;
struct points x2;
struct points yy2;
struct points z2;
struct points x3;
struct points yy3;
struct points z3;
struct points x4;
struct points yy4;
struct points z4;

// motor angle vars
struct angles leg1;
struct angles leg2;
struct angles leg3;
struct angles leg4;

// comms vars
double yaw;
double pitch;
double roll;
int state;
float x_dist;
float steer2;
double x_accel;
double y_accel;
double z_accel;
float kpp;
float kip;
float kdp;
float kpr;
float kir;
float kdr;
int mode;

float x_dist_commanded;
float steer2_commanded;
float kpp_commanded;
float kip_commanded;
float kdp_commanded;
float kpr_commanded;
float kir_commanded;
float kdr_commanded;
int mode_commanded = 0;

// comms globals
int comm_receive_step = 1;
int time_stamp;
uint16_t checksum;
char mnemonic[7];
data_type comm_data;
char double_buffer[8];
comm comm_results;
bool new_data;
char data_byte;
char telem_packet[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const int rs = 2, en = 3;
LiquidCrystal lcd(rs, en, 4, 5, 6, 7);

// AltSoftSerial SerialBT;//(13, 12); // RX | TX transmit 9, recieve 8

byte full[8] = {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
};

Bounce2::Button button1 = Bounce2::Button();
Bounce2::Button button2 = Bounce2::Button();
Bounce2::Button button3 = Bounce2::Button();
Bounce2::Button button4 = Bounce2::Button();

int X, Y;

void setup()
{
  pinMode(13, INPUT_PULLUP);
  // put your setup code here, to run once:
  button1.attach(9, INPUT_PULLUP);
  button2.attach(10, INPUT_PULLUP);
  button3.attach(11, INPUT_PULLUP);
  button4.attach(12, INPUT_PULLUP);
  button1.setPressedState(LOW);
  button2.setPressedState(LOW);
  button3.setPressedState(LOW);
  button4.setPressedState(LOW);
  //button1.interval(10);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  pinMode(8, OUTPUT);

  analogWrite(8, 80);

  lcd.createChar(0, full);
  lcd.begin(16, 2);
  lcd.print("First line");
  lcd.setCursor(0, 1);
  lcd.print("Second line");

  Serial.begin(115200);
  delay(3000);
  Serial4.begin(115200);
  lcd.clear();
}

void loop()
{
  t = millis();
  // put your main code here, to run repeatedly:
  X = analogRead(A0);
  Y = analogRead(A1);

  x_dist_commanded = map(X, 0, 1024, 99, -99);
  if ((X > -8) && (X < 5))
  {
    x_dist_commanded = 0;
  }
  steer2_commanded = map(Y, 0, 1024, 99, -99);
  if ((Y > -5) && (Y < 5))
  {
    steer2_commanded = 0;
  }

  if ((t % 10 == 0)&&(t_button != t))
  {
    button1.update();
    button2.update();
    button3.update();
    button4.update();
    t_button = t;
  }

  if (button1.pressed())
  {
    mode_commanded++;
    if (mode_commanded > 3){
      mode_commanded = 0;
    }
  }
  
  /*
    if (button2.isPressed() && !b2_send)
  {
    b2 = 1;
  }
  if (b2_send)
  {
    b2 = 0;
  }
  if (!button2.isPressed())
  {
    b2_send = 0;
  }

  if (button3.isPressed() && !b3_send)
  {
    b3 = 1;
  }
  if (b3_send)
  {
    b3 = 0;
  }
  if (!button3.isPressed())
  {
    b3_send = 0;
  }

  if (button4.isPressed() && !b4_send)
  {
    b4 = 1;
  }
  if (b4_send)
  {
    b4 = 0;
  }
  if (!button4.isPressed())
  {
    b4_send = 0;
  }
  */

#ifdef COMM_MODE_DESCRIPTIVE
  comms_send();
  comms_receive();
  comms_interpreter();
#endif

  // SerialBT.println(datapacket);
  // Serial.println(datapacket);

  /*
  //Serial.print(",");
  Serial.write((byte)Y);
  //Serial.print(",");
  Serial.write((byte)button1.isPressed());
  //Serial.print(",");
  Serial.write((byte)button2.isPressed());
  //Serial.print(",");
  Serial.write((byte)button3.isPressed());
  //Serial.print(",");
  Serial.write((byte)button4.isPressed());
  */
  if ((t % 50 == 0)&&(t_lcd !=t))
  {
    t_lcd = t;
    lcd.clear();

    // display joystick input
    lcd.setCursor(0, 0);
    lcd.print("X");
    if (x_dist_commanded >= 0)
    {
      lcd.setCursor(2, 0);
    }
    lcd.print(x_dist_commanded);
    lcd.setCursor(5, 0);
    lcd.print("Y");
    if (steer2_commanded >= 0)
    {
      lcd.setCursor(7, 0);
    }
    lcd.print(steer2_commanded);

    // display mode
    lcd.setCursor(11, 0);
    lcd.print("MODE");
    lcd.print(mode);

    // display translation values received back from kevin for modes 0 and 1
    if (mode == 0 || mode == 1)
    {
      lcd.setCursor(0, 1);
      lcd.print("x");
      if (xxx >= 0)
      {
        lcd.setCursor(2, 1);
      }
      lcd.print(xxx);
      lcd.setCursor(5, 1);
      lcd.print("y");
      if (yyy >= 0)
      {
        lcd.setCursor(7, 1);
      }
      lcd.print(yyy);
    }

    // display PID tuning data
    if (mode == 2 || mode == 3)
    {
      lcd.setCursor(0, 1);
      lcd.print("p");
      int_store = xxx / 100;
      lcd.print(int_store);
      lcd.print(".");
      if (xxx - (int_store * 100) < 10)
      {
        lcd.print(0);
      }
      lcd.print(xxx - (int_store * 100));

      lcd.setCursor(5, 1);
      lcd.print("r");
      int_store = yyy / 100;
      lcd.print(int_store);
      lcd.print(".");
      if (yyy - (int_store * 100) < 10)
      {
        lcd.print(0);
      }
      lcd.print(yyy - (int_store * 100));

      lcd.setCursor(15, 1);
      if (tune_mode == 0)
      {
        lcd.print("P");
      }
      if (tune_mode == 1)
      {
        lcd.print("I");
      }
      if (tune_mode == 2)
      {
        lcd.print("D");
      }
    }

    if (b1)
    {
      lcd.setCursor(13, 1);
      lcd.write(byte(0));
      b1 = 0;
    }
    if (button2.isPressed())
    {
      lcd.setCursor(14, 1);
      lcd.write(byte(0));
    }
    if (button3.isPressed())
    {
      lcd.setCursor(15, 1);
      lcd.write(byte(0));
    }
    if (button4.isPressed())
    {
      lcd.setCursor(15, 1);
      lcd.write(byte(0));
    }
  }
}
