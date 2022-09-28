package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ObjectiveController {

  RobotHardware robot;
  TeleOpObjectiveLogic OL;

  public ObjectiveController(RobotHardware r) {
    robot = r;
    OL = new TeleOpObjectiveLogic(robot, this);
  }

  public void inputs(Gamepad gamepad) {
    if (gamepad.a) { //intake
      OL.aButton();
    }
    if (gamepad.b) {
      OL.bButton();
    }
    if (gamepad.x) {
      OL.xButton();
    }
    if (gamepad.y) { //outtake
      OL.yButton();
    }
    if (gamepad.right_trigger > 0.1) {
      OL.speedDuck = 2;
    } else {
      OL.speedDuck = 1;
    }

    OL.setStates(gamepad);
  }
}
