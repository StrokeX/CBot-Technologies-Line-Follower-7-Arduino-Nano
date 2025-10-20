#include <CBotLF7.h>

LineRobot robot;

void setup() {
  robot.begin();

  robot.PLANS[0] = {0, 12, {30, 0 ,15}, 180, 255, -255, 170, {30, 0, 15}, 180, 170};
  robot.PLANS[1] = {1, 15, {30, 0 ,15}, 180, -255, 255, 170, {30, 0, 15}, 180, 700};

  robot.UsePlan = true;
  robot.Kps = 30;
  robot.Kis = 0;
  robot.Kds = 10;
}

void loop() {
  robot.update();
}
