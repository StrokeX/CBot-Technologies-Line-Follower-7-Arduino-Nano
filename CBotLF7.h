#ifndef CBotLF7_h
#define CBotLF7_h

#include <Arduino.h>

struct IndexStructure {
  int Action;
  int Line;
  int SensorCondition;
  int ScanPID[3];
  int ScanSpeed;
  int ActionLMSpeed;
  int ActionRMSpeed;
  int ActionDur;
  int ActionDel;
  int ThenPID[3];
  int ThenSpeed;
  int ThenDur;
};

class LineRobot {
  public:
    LineRobot();

    // User-modifiable variables
    IndexStructure PLANS[20];
    bool UsePlan;
    float Kps, Kis, Kds;
    float SensorSens;
    int ManualSens;
    int BaseSpeed;
    int MaxSpeed;

    void begin();
    void update();

  private:
    // Internal variables
    int Mult;
    float multiP, multiI, multiD;
    float lastError, error, totalError;
    int active;

    int SamplingTime;
    int LastTime;
    int CurIndex;
    bool Started;

    // Motor Pins
    int Rin1, Rin2;
    int Lin1, Lin2;

    // Buttons
    int StartBTN, CalBTN;

    // Sensors
    int S1, S2, S3, S4, S5, S6, S7;
    int SMin[7], SMax[7], SValues[7], SBit[7];

    // Internal functions
    void ReadSensors();
    void CalibrateSensors();
    void GetError();
    void DriveMotorLeft(int Speed);
    void DriveMotorRight(int Speed);
    void PID(int Kp, int Ki, int Kd);
    int ComapareSensor(int Con);
};

#endif
