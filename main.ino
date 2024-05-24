

//MOTOR CONTROL
int RPwm = 11;
int RDir = 13;
int LPwm = 3;
int LDir = 12;

//LED & BUZZER
int Buzzer = 9;
int ArduLed = 8;  // The led on the board to see visually when the mini sumo robot is operating.

//EDGE & CONTRAST SENSORS
int Redge = A0;
int Ledge = A1;

//TRIMPOTS
int SPD = A7;
int TRN = A6;

//MR45 SENSORS
int LSens = A4;
int RSens = A2;
int MSens = A3;

// DIPSWITCH & BUTTON
int Button = 10; // The button to start and turn off the mini sumo robot.
int DS1 = 5;
int DS2 = 6;
int DS3 = 7;

//VALUES
int Speed = 50;
int MaxSpeed = 50; // Idle Speed while no sensor giving data.
int TurnSpeed = 55; // Left and Right Forward Turning Speed
int EdgeTurn = 190; // Turning Time variable when minisumo sees white line
int Duration; // Turning Time at minisumo starting.
int LastValue = 5; // Last Value Variable for remembering last Opponent sensor sense.


void setup() {
  pinMode(LSens, INPUT); // Left Opponent Sensor Input
  pinMode(RSens, INPUT); // Right Opponent Sensor Input
  pinMode(MSens, INPUT); // Middle Opponent Sensor Input
  
  pinMode(Buzzer, OUTPUT); // Buzzer Declared as Output
  pinMode(ArduLed, OUTPUT); // ArduLed Declared as Output
  pinMode(Button, INPUT); // Button Declared as Input

  pinMode(RPwm, OUTPUT); // Four PWM Channels Declared as Output
  pinMode(RDir, OUTPUT);
  pinMode(LPwm, OUTPUT);
  pinMode(LDir, OUTPUT);

  digitalWrite(Buzzer, LOW); // Buzzer is initially starts in LOW --> silent.
  digitalWrite(ArduLed, LOW); 
  
  digitalWrite(DS1, HIGH); // 3 Dipswitch Pin Pullups Made
  digitalWrite(DS2, HIGH);
  digitalWrite(DS3, HIGH);
  
  Serial.begin(9600);
  
  tone(Buzzer, 523, 300);
  delay(300);
  noTone(Buzzer);
}


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
  digitalWrite(RPwm, LOW);  // INITIALIZE MOTORS (STOP)
  digitalWrite(LPwm, LOW);  // INITIALIZE MOTORS (STOP)

/// Button Press Detection and Initialization ///
  
  if (digitalRead(Button) == 1) { // If button is pressed at the first start.
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
        delay(300);
        noTone(Buzzer);
        digitalWrite(ArduLed, HIGH);
        delay(300);
        digitalWrite(ArduLed, LOW);
      }
    }
  }
  
  tone(Buzzer, 440, 200);
  delay(200);
  tone(Buzzer, 494, 500);
  delay(500);
  noTone(Buzzer);

  Serial.println("Button Press Waited");
  Set_Motor(0, 0, 1);

  /// Sensor Control While Waiting The Button Press ///
  if (digitalRead(MSens) == LOW || digitalRead(RSens) == LOW || digitalRead(LSens) == LOW ||
      analogRead(Redge) < 500 || analogRead(Ledge) < 500) {
    digitalWrite(ArduLed, HIGH);
  } else {
    digitalWrite(ArduLed, LOW);
  }
//  ##### Button Press Detection and Initialization ##########
  //STARTS A 5 SECOND ROUTINE
  if (digitalRead(Button) == 1) {
    Duration = (analogRead(TRN) / 4); // Duration variable based on TRN (A6) trimpot
    Serial.println("5 Sec Routine Started");
    
    for (int i = 0; i < 5; i++) {
      Set_Motor(0, 0, 1);
      digitalWrite(ArduLed, HIGH);
      tone(Buzzer, 523, 300);
      delay(500);
      noTone(Buzzer);
      digitalWrite(ArduLed, LOW);
      delay(200);
    }

    if (digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 1) {
      Serial.print("LEFT TURN");
      Set_Motor(-100, 100, 180); //
    } else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0) {
      Serial.print("MIDDLE DIRECT");
      Set_Motor(80, 80, 2);
    } else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0) {
      Serial.print("Sag");
      Set_Motor(100, -100, 180);
    } else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0) {
      Serial.print("Left Circle");
      Set_Motor(100, 36, 650);
    } else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 1) {
      Serial.print("Right Circle");
      Set_Motor(36, 100, 650);
    } else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0) {
      Serial.print("Reverse 180");
      Set_Motor(-100, 100, 150);
      delay(100);
    }
    Serial.print("OK");
    digitalWrite(Buzzer, LOW);
  }

  /// Edge Sensor Control Routine ///

  digitalWrite(ArduLed, LOW);

  if (analogRead(Ledge) < 100 && analogRead(Redge) > 100) {
    digitalWrite(Buzzer, LOW);
    digitalWrite(ArduLed, HIGH);
    Set_Motor(-100, -100, 35); // Backward
    Set_Motor(-100, 100, EdgeTurn); // Left Backward, Right Forward, Turning Time Based on ETRN Trimpot
    LastValue = 5;
  } else if (analogRead(Ledge) > 100 && analogRead(Redge) < 100) {
    digitalWrite(Buzzer, LOW);
    digitalWrite(ArduLed, HIGH);
    Set_Motor(-100, -100, 35); // Back 35 Milliseconds
    Set_Motor(100, -100, EdgeTurn); // Right Backward, Left Forward, Turning Time Based on ETRN Trimpot
    LastValue = 5;
  } else if (analogRead(Ledge) < 100 && analogRead(Redge) < 100) {
    digitalWrite(Buzzer, LOW);
    digitalWrite(ArduLed, HIGH);
    Set_Motor(-100, -100, 35); // Back 35 Milliseconds
    Set_Motor(100, -100, EdgeTurn); // Right Backward, Left Forward, Turning Time Based on ETRN Trimpot
    LastValue = 5;
  } else {
    
    /// MR45 Sensor Control Routine ///

    digitalWrite(Buzzer, LOW);
    
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
      Speed = (analogRead(SPD) / 10.3);
      Speed = 100 - Speed;
      
      if (LastValue == 5) {
        Set_Motor(70, 70, 1); // Forward, Based on SPD (A7) Trimpot
      } else if (LastValue == 7) {
        Set_Motor(-20, 100, 2); // Left Turning Based on SPD (A7) Trimpot
      } else if (LastValue == 3) {
        Set_Motor(100, -20, 2); // Right Turning Based on SPD (A7) Trimpot
      }
    }
  }
}
