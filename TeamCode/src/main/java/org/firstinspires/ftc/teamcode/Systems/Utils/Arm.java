package org.firstinspires.ftc.teamcode.Systems.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class is used to control the arm on the robot
 * <br />The PID (just PD) Weights are constants inside of the class, so that's where to change those.
 */
public class Arm {

  DcMotor joint1m1, joint1m2, joint2;
  Servo clawAligner;

  double JOINT1P = .5, JOINT1D = .5;
  double JOINT2P = .5, JOINT2D = .5;
  double CLAWP = .5, CLAWD = .5;

  double currentJoint1Angle, currentJoint2Angle = 0;
  double targetJoint1Angle, targetJoint2Angle = 0;
  double previousJoint1Angle, previousJoint2Angle = 0;

  // claw aligner angle only needs a target because we basically just call servo.setPosition() on it every loop bc we cant do it with a pid.
  double targetClawAlignerAngle = 0;


  public Arm(
    DcMotor joint1m1,
    DcMotor joint1m2,
    DcMotor joint2,
    Servo clawAligner
  ) {
    this.joint1m1 = joint1m1;
    this.joint1m2 = joint1m2;
    this.joint2 = joint2;
    this.clawAligner = clawAligner;
  }

  /**
   * Moves the arm to a certain position. x^2 + y^2 should not be > 2. It handles that exception, but not well.
   * @param x the horizontal position of the arm, max 2
   * @param y the wanted vertical position of the arm. -2 is the lowest position, 2 is the highest position
   * @return the value of c <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   */
  private double calculateC(double x, double y) {
    double answer = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    if (answer > 2) {
      double temp = Math.sqrt(Math.pow(answer - 2, 2) / 2);
      return calculateC(x - temp, y - temp);
    } else {
      return answer;
    }
  }

  /**
   * calculates theta <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @param c the value of c <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @return the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a> in radians
   */
  private double calculateTheta(double c) {
    double theta = Math.acos(-(c * c / 2 - 1));
    return theta;
  }

  /**
   * calculates theta <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>.
   * magnitude of (x,y) <= 2
   * @param x the wanted horizontal position of the arm, max 2
   * @param y the wanted vertical position of the arm, max 2
   * @return the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a> in radians
   */
  public double calculatePhi(double x, double y) {
    double phi = Math.atan(y / x);
    return phi;
  }

  /**
   * calculates the angle of the first joint <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @param theta the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @param phi the value of phi <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @return the angle of the first joint <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a> in radians (0 to pi)
   */
  public double calculateJoint1Angle(double theta, double phi) {
    double joint1Angle = Math.PI - (theta / 2) + phi;
    return joint1Angle;
  }

  /**
   * calculates the angle of the second joint <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @param theta the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @return the angle of the second joint <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a> in radians (0 to pi)
   */
  public double calculateJoint2Angle(double theta) {
    double joint2Angle = theta;
    return joint2Angle;
  }

  /**
   * calculates the angle of the claw aligner servo to make it perfectly horizontal with the ground
   * @param theta the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @param phi the value of phi <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   * @return the angle of the claw aligner servo to make it perfectly horizontal with the ground in radians
   */
  public double calculateClawAngle(double theta, double phi) {
    double clawAngle = phi - (Math.PI - theta / 2) + theta;
    return clawAngle;
  }

  public void setTargetJoint1Angle(double targetJoint1Angle) {
    this.targetJoint1Angle = targetJoint1Angle;
  }

  public void setTargetJoint2Angle(double targetJoint2Angle) {
    this.targetJoint2Angle = targetJoint2Angle;
  }

  public void setTargetClawAlignerAngle(double targetClawAlignerAngle) {
    this.targetClawAlignerAngle = targetClawAlignerAngle;
  }

  public void updateAngles() {
    currentJoint1Angle = ticksToRadians( joint1m1.getCurrentPosition()) ;
    currentJoint2Angle = ticksToRadians(joint2.getCurrentPosition() );
  }

  private static double ticksToRadians(double ticks) {
      return 2*Math.PI*ticks/145.1;
  }
  private static double radiansToTicks(double radians) {
      return radians*145.1/(2*Math.PI);
  }
}
