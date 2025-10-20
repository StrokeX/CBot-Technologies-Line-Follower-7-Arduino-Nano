#include "CBotLF7.h"

LineRobot::LineRobot() {
  // Default values
  BaseSpeed = 180;
  MaxSpeed = 255;
  Mult = 100;
  multiP = 2;
  multiI = 1;
  multiD = 1;
  lastError = 0;
  error = 0;
  totalError = 0;
  active = 0;

  Kps = 30;
  Kis = 0;
  Kds = 10;
  SamplingTime = 15;
  LastTime = 0;

  Rin1 = 10;
  Rin2 = 11;
  Lin1 = 5;
  Lin2 = 6;

  StartBTN = 12;
  CalBTN = 2;
  Started = false;

  S1 = A6;
  S2 = A5;
  S3 = A4;
  S4 = A3;
  S5 = A2;
  S6 = A1;
  S7 = A0;

  CurIndex = 0;
  UsePlan = true;
}

void LineRobot::begin() {
  Serial.begin(9600);

  pinMode(Rin1, OUTPUT);
  pinMode(Rin2, OUTPUT);
  pinMode(Lin1, OUTPUT);
  pinMode(Lin2, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);

  pinMode(StartBTN, INPUT_PULLUP);
  pinMode(CalBTN, INPUT_PULLUP);
}

void LineRobot::update() {
  if (digitalRead(StartBTN) == LOW) {
    while (digitalRead(StartBTN) == LOW) {}
    if (Started) {
      analogWrite(Rin1, 255);
      analogWrite(Rin2, 255);
      analogWrite(Lin1, 255);
      analogWrite(Lin2, 255);
      delay(500);
    }
    Started = !Started;
    delay(250);
  }

  if (digitalRead(CalBTN) == LOW) CalibrateSensors();

  if (Started) {
    if (UsePlan) {
      BaseSpeed = PLANS[CurIndex].ScanSpeed;

      if (ComapareSensor(PLANS[CurIndex].SensorCondition)) {
        DriveMotorLeft(PLANS[CurIndex].ActionLMSpeed);
        DriveMotorRight(PLANS[CurIndex].ActionRMSpeed);
        delay(PLANS[CurIndex].ActionDur);

        BaseSpeed = PLANS[CurIndex].ThenSpeed;
        error = 0;
        lastError = 0;

        unsigned long startTime = millis();
        while (millis() - startTime < PLANS[CurIndex].ThenDur) {
          PID(PLANS[CurIndex].ThenPID[0], PLANS[CurIndex].ThenPID[1], PLANS[CurIndex].ThenPID[2]);
        }

        if (PLANS[CurIndex].Action == 1) {
          Started = false;
          analogWrite(Rin1, 255);
          analogWrite(Rin2, 255);
          analogWrite(Lin1, 255);
          analogWrite(Lin2, 255);
          CurIndex = 0;
          delay(500);
        } else {
          CurIndex++;
          error = 0;
          lastError = 0;
        }
      }

      PID(PLANS[CurIndex].ScanPID[0], PLANS[CurIndex].ScanPID[1], PLANS[CurIndex].ScanPID[2]);
    } else {
      PID(Kps, Kis, Kds);
    }
  } else {
    DriveMotorLeft(0);
    DriveMotorRight(0);
  }
}

void LineRobot::ReadSensors() {
  active = 0;
  SValues[0] = analogRead(S1);
  SValues[1] = analogRead(S2);
  SValues[2] = analogRead(S3);
  SValues[3] = analogRead(S4);
  SValues[4] = analogRead(S5);
  SValues[5] = analogRead(S6);
  SValues[6] = analogRead(S7);

  for (int i = 0; i < 7; i++) {
    if (SValues[i] < 600) {
      SBit[i] = 1;
      active++;
    } else {
      SBit[i] = 0;
    }
  }
}

void LineRobot::CalibrateSensors() {
  delay(500);
  analogWrite(Rin1, 150);
  analogWrite(Rin2, 0);
  analogWrite(Lin1, 0);
  analogWrite(Lin2, 150);

  for (int i = 0; i < 7; i++) {
    SMax[i] = 1023;
    SMin[i] = 0;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < 7; i++) {
      ReadSensors();
      int value = SValues[i];
      if (value < SMin[i]) SMin[i] = value;
      if (value > SMax[i]) SMax[i] = value;
    }
  }

  DriveMotorLeft(0);
  DriveMotorRight(0);
}

void LineRobot::GetError() {
  float weight = 0, sensorSum = 0;

  for (int i = 0; i < 7; i++) {
    if (SBit[i] == 1) {
      weight += (i - (7 / 2));
      sensorSum += 1;
    }
  }

  if (sensorSum > 0) error = (weight / sensorSum) * Mult;
  else error = lastError;
}

void LineRobot::DriveMotorLeft(int Speed) {
  if (Speed >= 0) {
    analogWrite(Lin1, abs(Speed));
    analogWrite(Lin2, 0);
  } else {
    analogWrite(Lin1, 0);
    analogWrite(Lin2, abs(Speed));
  }
}

void LineRobot::DriveMotorRight(int Speed) {
  if (Speed >= 0) {
    analogWrite(Rin1, abs(Speed));
    analogWrite(Rin2, 0);
  } else {
    analogWrite(Rin1, 0);
    analogWrite(Rin2, abs(Speed));
  }
}

void LineRobot::PID(int Kp, int Ki, int Kd) {
  ReadSensors();
  GetError();

  int NOW = millis();
  int Interval = NOW - LastTime;

  if (active == 0) {
    if (lastError > 0) {
      DriveMotorRight(-BaseSpeed);
      DriveMotorLeft(BaseSpeed);
    } else {
      DriveMotorRight(BaseSpeed);
      DriveMotorLeft(-BaseSpeed);
    }
  } else {
    if (Interval >= SamplingTime) {
      float P = error;
      float I = totalError;
      float D = error - lastError;

      float Pf = (Kp / pow(10, multiP)) * P;
      float If = (Ki / pow(10, multiI)) * I;
      float Df = (Kd / pow(10, multiD)) * D;

      float pidTotal = Pf + If + Df;

      int leftSpeed = BaseSpeed + pidTotal;
      int rightSpeed = BaseSpeed - pidTotal;

      leftSpeed = constrain(leftSpeed, -MaxSpeed, MaxSpeed);
      rightSpeed = constrain(rightSpeed, -MaxSpeed, MaxSpeed);

      DriveMotorLeft(leftSpeed);
      DriveMotorRight(rightSpeed);

      lastError = error;
      totalError += error;
      LastTime = NOW;
    }
  }
}

int LineRobot::ComapareSensor(int Con) {
  ReadSensors();
  switch (Con) {
    case 0: return true;
    case 1: return (active == 7);
    case 2: return (active == 0);
    case 3: return (SBit[6] == 1);
    case 4: return (SBit[5] == 1);
    case 5: return (SBit[4] == 1);
    case 6: return (SBit[3] == 1);
    case 7: return (SBit[2] == 1);
    case 8: return (SBit[1] == 1);
    case 9: return (SBit[0] == 1);
    case 10: return ((SBit[3] == 1) && (SBit[6] == 1));
    case 11: return ((SBit[3] == 1) && (SBit[6] == 1 || SBit[5] == 1));
    case 12: return ((SBit[2] == 1 || SBit[3] == 1 || SBit[4] == 1) && (SBit[6] == 1));
    case 13: return ((SBit[3] == 1) && (SBit[0] == 1));
    case 14: return ((SBit[3] == 1) && (SBit[0] == 1 || SBit[1] == 1));
    case 15: return ((SBit[2] == 1 || SBit[3] == 1 || SBit[4] == 1) && (SBit[0] == 1));
    case 16: return ((SBit[0] == 1) && (SBit[6] == 1));
    case 17: return ((SBit[0] == 1 || SBit[1] == 1) && (SBit[5] == 1 || SBit[6] == 1));
    case 18: return ((SBit[0] == 1 || SBit[1] == 1 || SBit[3] == 1) && (SBit[4] == 1 || SBit[5] == 1 || SBit[6] == 1));
  }
  return false;
}
