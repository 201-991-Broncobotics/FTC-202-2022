package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class TeleOpObjectiveLogic {

  RobotHardware robot;
  ObjectiveController objective;

  //Variables
  private int servoPosition = 0;

  private boolean aPrev = false;
  private boolean bPrev = false;
  private boolean xPrev = false;
  private boolean yPrev = false;

  private boolean dUpPrev = false;
  private boolean dDownPrev = false;

  private boolean aMotor = false;
  private boolean bMotor = false;
  private boolean xMotor = false;
  private boolean yMotor = false;

  public int speedDuck = 1;

  private boolean ArmActive = false;

  private double rPosition = 1;

  //Variables

  public TeleOpObjectiveLogic(RobotHardware r, ObjectiveController o) {
    robot = r;
    objective = o;
    robot.telemetry.addData("here", "Here2");
    robot.telemetry.update();
  }

  public void aButton() {
    if (!aPrev) {
      aMotor = !aMotor;
      if (yMotor) {
        yMotor = false;
      }
    }
  }

  public void bButton() {
    if (!bPrev) {
      bMotor = !bMotor;
      if (xMotor) {
        xMotor = false;
      }
    }
  }

  public void xButton() {
    if (!xPrev) {
      xMotor = !xMotor;
      if (bMotor) {
        bMotor = false;
      }
    }
  }

  public void yButton() {
    if (!yPrev) {
      yMotor = !yMotor;
      if (aMotor) {
        aMotor = false;
      }
    }
  }

  public void setStates(Gamepad g) {
    aPrev = g.a;
    bPrev = g.b;
    xPrev = g.x;
    yPrev = g.y;
    dUpPrev = g.dpad_up;
    dDownPrev = g.dpad_down;
    if (g.left_stick_y > -0.1 && g.left_stick_y < 0.1) {
      ArmActive = false;
    }
  }
}
