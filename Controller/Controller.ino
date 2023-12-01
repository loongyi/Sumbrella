
//#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>

#define RED 22
#define BLUE 24
#define GREEN 23

int a = 2;  //For displaying segment "b"
int b = 3;  //For displaying segment "b"
int c = 4;  //For displaying segment "c"
int d = 5;  //For displaying segment "d"
int e = 6;  //For displaying segment "e"
int f = 7;  //For displaying segment "f"
int g = 8;  //For displaying segment "g"
int SW = 12;
int GO = 11;

int gesture = -1;
int oldGestureValue = -1;
const int ledPin = LED_BUILTIN;  // pin to use for the LED

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(a, OUTPUT);  //A
  pinMode(b, OUTPUT);  //B
  pinMode(c, OUTPUT);  //C
  pinMode(d, OUTPUT);  //D
  pinMode(e, OUTPUT);  //E
  pinMode(f, OUTPUT);  //F
  pinMode(g, OUTPUT);  //G

  pinMode(SW, INPUT_PULLUP);
  pinMode(GO, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
  digitalWrite(GREEN, HIGH);
  turnOff();

  if (!APDS.begin()) {
    Serial.println("* Error initializing APDS9960 sensor!");
  }

  APDS.setGestureSensitivity(80);
  Serial.println(11);
}
int i = 0, msg = 0, oldmsg = 0, manual = 1;

void loop() {
  // put your main code here, to run repeatedly:

  if (manual > 0) {
    gesture = gestureDetectection();
    if ((gesture == 4) || (!digitalRead(SW))) {
      turnOff();
      i++;
      if (i > 10) { i = 0; }
      led();
    }
    if (gesture == 3) {
      turnOff();
      i--;
      if (i < 0) { i = 0; }
      led();
    }
    if (0 <= i < 10) {
      displayDigit(i);
    }
    if (i == 10) {
      turnOff();
      dispAUTO();
      
    }
    delay(100);

    if (!digitalRead(GO)) {
      digitalWrite(BLUE, LOW);
      Serial.println(i);
      if (i == 10) {
        digitalWrite(GREEN, LOW);
        manual = 0;
      } else {
        digitalWrite(GREEN, HIGH);
      }

      delay(300);
      digitalWrite(BLUE, HIGH);
    }
    // else{ // no command
    //   Serial.println(11);
    // }
  }
  if ((!digitalRead(GO)) && (manual == 0)) {
    turnOff();
    dispCONT();
    Serial.println(11);
    digitalWrite(GREEN, HIGH);
    manual = 1;
    Serial.flush();
    delay(300);
  }
  if ((Serial.available()) && (manual==0)) {
    //will only receive messages when controller AUTO is on. 0 - 9 only, send 10 for manual
    msg = Serial.readStringUntil('\n').toInt();
    //manual = 0;
    if (msg != oldmsg) {
      turnOff();
      oldmsg = msg;
      delay(10);
    }
    if (msg == 10) {
      manual = 2;
      dispCONT();
      digitalWrite(GREEN, HIGH);
      delay(500);
      Serial.flush();
    } else {
      displayDigit(msg);
    }
    if (msg == 11){
      turnOff();
      dispAUTO();
    }
  }
}

void led() {
  digitalWrite(ledPin, HIGH);
  digitalWrite(RED, LOW);
  delay(200);
  digitalWrite(ledPin, LOW);
  digitalWrite(RED, HIGH);
}

void dispAUTO() {
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}

void dispCONT() {
  digitalWrite(a, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
}

void displayDigit(int digit) {
  //Conditions for displaying segment a
  if (digit != 1 && digit != 4)
    digitalWrite(a, HIGH);

  //Conditions for displaying segment b
  if (digit != 5 && digit != 6)
    digitalWrite(b, HIGH);

  //Conditions for displaying segment c
  if (digit != 2)
    digitalWrite(c, HIGH);

  //Conditions for displaying segment d
  if (digit != 1 && digit != 4 && digit != 7)
    digitalWrite(d, HIGH);

  //Conditions for displaying segment e
  if (digit == 2 || digit == 6 || digit == 8 || digit == 0)
    digitalWrite(e, HIGH);

  //Conditions for displaying segment f
  if (digit != 1 && digit != 2 && digit != 3 && digit != 7)
    digitalWrite(f, HIGH);

  if (digit != 0 && digit != 1 && digit != 7)
    digitalWrite(g, HIGH);
}
void turnOff() {
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}

int gestureDetectection() {
  if (APDS.gestureAvailable()) {
    gesture = APDS.readGesture();

    switch (gesture) {
      case GESTURE_UP:
        //Serial.println("- UP gesture detected");
        return 1;
        break;
      case GESTURE_DOWN:
        //Serial.println("- DOWN gesture detected");
        return 2;
        break;
      case GESTURE_LEFT:
        //Serial.println("- LEFT gesture detected");
        return 3;
        break;
      case GESTURE_RIGHT:
        //Serial.println("- RIGHT gesture detected");
        return 4;
        break;
      default:
        //Serial.println("- No gesture detected");
        return 0;
        break;
    }
  }
  //return gesture;
}