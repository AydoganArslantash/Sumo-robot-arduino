// NO LONGER IN USE DO NOT USE IT!

// MOTOR CONTROL
int RPwm = 11;
int RDir = 13;
int LPwm = 3;
int LDir = 12;

// LED & BUZZER
int Buzzer = 9;
int ArduLed = 8;

// EDGE & CONTRAST SENSORS
int Redge = A0;
int Ledge = A1;

// TRIMPOTS
int SPD = A7;
int TRN = A6;

// OPPONENT SENSORS
int LSens = A4;
int RSens = A2;
int MSens = A3;
int LFSens = A5;
int RFSens = 4;

// DIPSWITCH & BUTTON
int Button = 10; 
int DS1 = 5;
int DS2 = 6;
int DS3 = 7;

// VALUES
int Speed = 50;
int MaxSpeed = 50; // Idle Speed while no sensor giving data.
int TurnSpeed = 55; // Left and Right Forward Turning Speed
int EdgeTurn = 190; // Turning Time variable when minisumo sees white line
int Duration; // Turning Time at minisumo starting.
int LastValue = 5; // Last Value Variable for remembering last Opponent sensor sense.

void setup() {
    // Initialize pin modes
    pinMode(LSens, INPUT); // Left Opponent Sensor Input
    pinMode(RSens, INPUT); // Right Opponent Sensor Input
    pinMode(MSens, INPUT); // Middle Opponent Sensor Input

    pinMode(Buzzer, OUTPUT); // Buzzer Declared as Output
    pinMode(ArduLed, OUTPUT); // Arduino Mode Led Declared as Output
    pinMode(Button, INPUT); // Button Declared as Input
    
    pinMode(RPwm, OUTPUT); // Four PWM Channel Declared as Output
    pinMode(RDir, OUTPUT);
    pinMode(LPwm, OUTPUT);
    pinMode(LDir, OUTPUT);
    
    digitalWrite(Buzzer, LOW); // Buzzer Pin Made Low for Silence
    digitalWrite(ArduLed, LOW); // Arduino Mode Led Made Low
    
    digitalWrite(DS1, HIGH); // 3 Dipswitch Pin Pullups Made
    digitalWrite(DS2, HIGH);
    digitalWrite(DS3, HIGH);
    
    digitalWrite(RFSens, HIGH);
    digitalWrite(MSens, HIGH);
    
    Serial.begin(9600);
    
    tone(Buzzer, 523, 300); // Initial tone
    delay(300);
    noTone(Buzzer);
}

// Motor Control Function
void Set_Motor(float Lval, float Rval, int timex) {
    Lval = Lval * 2.5;
    Rval = Rval * 2.5;
    if (Lval >= 0) {
        analogWrite(LPwm, Lval);
        digitalWrite(LDir, LOW);
    } else {
        Lval = abs(Lval);
        digitalWrite(LDir, HIGH);
        analogWrite(LPwm, Lval);
    }
    if (Rval >= 0) {
        analogWrite(RPwm, Rval);
        digitalWrite(RDir, HIGH);
    } else {
        Rval = abs(Rval);
        digitalWrite(RDir, LOW);
        analogWrite(RPwm, Rval);
    }
    delay(timex);
}

void loop() {
    digitalWrite(RPwm, LOW);
    digitalWrite(LPwm, LOW);
    if (digitalRead(Button) == 1) { // If button is pressed at the first start
        tone(Buzzer, 18, 100); // Pin, Frequency, Duration
        while (1) {
            if (digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0) {
                Serial.print("Board Test");
                Set_Motor(10, 10, 50);
                Set_Motor(100, 100, 1000);
                Set_Motor(0, 0, 1000);
                Set_Motor(-10, -10, 50);
                Set_Motor(-100, -100, 1000);
                Set_Motor(0, 0, 1000);
                tone(Buzzer, 18, 300);
                digitalWrite(ArduLed, HIGH); delay(300); digitalWrite(ArduLed, LOW);
            }
        }
    }
    tone(Buzzer, 440, 200);
    tone(Buzzer, 494, 500);

Wait:
    Serial.println("Button Press Waited");
    Set_Motor(0, 0, 1);
    // Sensor Control While Waiting The Button Press
    if (digitalRead(MSens) == LOW || digitalRead(RSens) == LOW || digitalRead(LSens) == LOW || analogRead(Redge) < 500 || analogRead(Ledge) < 500) {
        digitalWrite(ArduLed, HIGH);
    } else {
        digitalWrite(ArduLed, LOW);
    }
    if (digitalRead(Button) == 1) {
        Duration = (analogRead(TRN) / 4); // Duration variable based on TRN (A6) trimpot
        Serial.println("5 Sec Routine Started");
        delay(500);
        noTone(Buzzer);
        digitalWrite(ArduLed, LOW);
        delay(200);
        if (digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 1) {
            Serial.print("LEFT TURN");
            Set_Motor(-100, 100, 180); // Turn left
        } else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0) {
            Serial.print("MIDDLE DIRECT");
            Set_Motor(80, 80, 2); // Move forward
        } else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0) {
            Serial.print("Right Turn");
            Set_Motor(100, -100, 180); // Turn right
        } else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0) {
            Serial.print("Left Circle");
            Set_Motor(100, 36, 650); // Circle left
        } else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 1) {
            Serial.print("Right Circle");
            Set_Motor(36, 100, 650); // Circle right
        } else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0) {
            Serial.print("Reverse 180");
            Set_Motor(-100, 100, 150); // Reverse 180
            delay(100);
        }
        Serial.print("OK");
        digitalWrite(Buzzer, LOW);
        goto Start;
    }

Start:
    // Edge Sensor Control Routine
    digitalWrite(ArduLed, LOW);
    if (analogRead(Ledge) < 100 && analogRead(Redge) > 100) {
        digitalWrite(Buzzer, LOW);
        digitalWrite(ArduLed, HIGH);
        Set_Motor(-100, -100, 35); // Move back
        Set_Motor(-100, 100, EdgeTurn); // Left backward, right forward
        LastValue = 5;
    } else if (analogRead(Ledge) > 100 && analogRead(Redge) < 100) {
        digitalWrite(Buzzer, LOW);
        digitalWrite(ArduLed, HIGH);
        Set_Motor(-100, -100, 35); // Move back
        Set_Motor(100, -100, EdgeTurn); // Right backward, left forward
        LastValue = 5;
    } else if (analogRead(Ledge) < 100 && analogRead(Redge) < 100) {
        digitalWrite(Buzzer, LOW);
        digitalWrite(ArduLed, HIGH);
        Set_Motor(-100, -100, 35); // Move back
        Set_Motor(100, -100, EdgeTurn); // Right backward, left forward
        LastValue = 5;
    } else {
        // Opponent Sensor Control Routine
        digitalWrite(Buzzer, LOW);
    }
    if (digitalRead(MSens) == LOW) {
        Set_Motor(100, 100, 1);
        digitalWrite(Buzzer, HIGH);
        LastValue = 5;
    } else if (digitalRead(LSens) == LOW) {
        Set_Motor(-100, 100, 1);
        digitalWrite(Buzzer, HIGH);
        LastValue = 7;
    } else if (digitalRead(RSens) == LOW) {
        Set_Motor(100, -100, 1);
        digitalWrite(Buzzer, HIGH);
        LastValue = 3;
    } else {
        digitalWrite(Buzzer, LOW);
        if (LastValue == 5) {
            Set_Motor(100, 100, 1);
        } else if (LastValue == 3) {
            Set_Motor(100, -100, 1);
        } else if (LastValue == 7) {
            Set_Motor(-100, 100, 1);
        } else {
            Set_Motor(MaxSpeed, MaxSpeed, 1);
        }
    }
}
