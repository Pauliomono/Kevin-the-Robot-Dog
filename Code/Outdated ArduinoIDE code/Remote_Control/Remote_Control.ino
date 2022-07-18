#include LiquidCrystal.h
#include Bounce2.h
#include SoftwareSerial.h

String datapacket;

const int rs = 2, en = 3;
LiquidCrystal lcd(rs, en, 4, 11, 6, 7);

SoftwareSerial SerialBT(13,12); // RX | TX

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


void setup() {
  // put your setup code here, to run once:
  button1.attach(10, INPUT_PULLUP);
  button2.attach(9, INPUT_PULLUP);
  button3.attach(8, INPUT_PULLUP);
  button4.attach(0, INPUT_PULLUP);
  button1.setPressedState(LOW); 
  button2.setPressedState(LOW); 
  button3.setPressedState(LOW); 
  button4.setPressedState(LOW); 
  
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  pinMode(5,OUTPUT);
  
  analogWrite(5,80);


  lcd.createChar(0, full);
  lcd.begin(16, 2);
  lcd.print("First line");
  lcd.setCursor(0,1);
  lcd.print("Second line");

  Serial.begin(9600);
  delay(3000);
  SerialBT.begin(38400);
  lcd.clear();

}

void loop() {
  // put your main code here, to run repeatedly:
    
  X = analogRead(A3);
  Y = analogRead(A2);

  X = map(X, 0, 1024, 100, -100);
  if ((X > -8)&&(X < 5)){
    X = 0;
  }
  Y = map(Y, 0, 1024, 100, -100);
  if ((Y > -5)&&(Y < 5)){
    Y = 0;
  }
  
  button1.update();
  button2.update();
  button3.update();
  button4.update();
/*
  sequence[0] = X;
  sequence[1] = Y;
  sequence[2] = button1.isPressed();
  sequence[3] = button2.isPressed();
  sequence[4] = button3.isPressed();
  sequence[5] = button4.isPressed();
  Serial.write(sequence, sizeof(sequence));
  */
  datapacket = "x";
  datapacket += num2str(X);
  datapacket += num2str(Y);
  datapacket += bool2str(button1.pressed());
  datapacket += bool2str(button2.pressed());
  datapacket += bool2str(button3.pressed());
  datapacket += bool2str(button4.pressed());
  datapacket += "n";
  
  SerialBT.println(datapacket);

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
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(X);
  lcd.setCursor(8,0);
  lcd.print(Y);
  
  if (button1.isPressed()){
    lcd.setCursor(0,1);
    lcd.write(byte(0));
  }
  if (button2.isPressed()){
    lcd.setCursor(7,1);
    lcd.write(byte(0));
  }
  if (button3.isPressed()){
    lcd.setCursor(14,1);
    lcd.write(byte(0));
  }
  if (button4.isPressed()){
    lcd.setCursor(15,1);
    lcd.write(byte(0));
  }
  
  delay(100);

}

String num2str(int X){
  String packet;
  if (X >=0){
    packet = "1";
  }
  else{
    packet = "0";
  }
  if (abs(X) < 10){
    packet += "00";
  }
  else if (abs(X) < 100){
    packet += "0";
  }

  packet += abs(X);

   return(packet);
}

String bool2str(bool X){
  String packet;
  if (X){
    packet = "1";
  }
  else{
    packet = "0";
  }

   return(packet);
}
