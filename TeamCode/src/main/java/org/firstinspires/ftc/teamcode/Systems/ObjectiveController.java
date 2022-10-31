package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ObjectiveController {

  RobotHardware robot;
  TeleOpObjectiveLogic OL;

  public ObjectiveController(RobotHardware r) {
    robot = r;
    OL = new TeleOpObjectiveLogic(robot, this);
  }

  public void scuffedArm(Gamepad gamepad) {
    double LY = -gamepad.left_stick_y;
    double RY = -gamepad.right_stick_y;
    
    robot.joint1m1.setPower(.25 * LY);
    robot.joint1m2.setPower(-.25 * LY);
    robot.joint2.setPower(.25 * RY);
    
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
