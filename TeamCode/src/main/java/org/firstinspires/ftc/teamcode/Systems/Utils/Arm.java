package org.firstinspires.ftc.teamcode.Systems.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class is used to control the arm on the robot
 * <br />
 * The PID (just P) Weights are constants inside of the class, so that's where
 * to change those.
 */
public class Arm {

  public DcMotor joint1m1, joint1m2, joint2;
  public Servo clawAligner;

  private double JOINT1P = ticksToRadians(.5);
  private double JOINT2P = ticksToRadians(.5);
  // double CLAWP = .5, CLAWD = .5;

  private double currentJoint1Angle, currentJoint2Angle = 0;
  private double targetJoint1Angle, targetJoint2Angle = 0;
  private double previousJoint1Angle, previousJoint2Angle = 0;

  private double previousTime, currentTime = 0;

  public Arm(
      DcMotor joint1m1,
      DcMotor joint1m2,
      DcMotor joint2,
      Servo clawAligner) {
    this.joint1m1 = joint1m1;
    this.joint1m2 = joint1m2;
    this.joint2 = joint2;
    this.clawAligner = clawAligner;
  }

  /**
   * @deprecated don't use, only for testing
   */
  public Arm() {
  }

  /**
   * function to be called in a while loop. It will move the arm to the target
   * angles set by other function calls. <br />
   * Important: does not block the thread
   */
  public void loop() {
    updateAngles();
    double joint1power = getJoint1Power();
    double joint2power = getJoint2Power();
    double theta = getCurrentTheta();
    double phi = getCurrentPhi();
    double clawAngle = calculateClawAngle(theta, phi);

    setJoint1Power(joint1power);
    setJoint2Power(joint2power);
    setClawAlignerTargetAngle(clawAngle);
  }

  /**
   * ONLY AUTONOMOUS USE. Moves to an (x,y) position where the magnitude of (x,y)
   * <= 2
   *
   * @param x 0 <= x <= 2, where 0 is as close to the initial joint as it can get
   *          and 2 is as far out horizontally as it can get
   * @param y -2 <= y <= 2, where 2 is as high as it can go, and -2 is as low as
   *          it can go.
   */
  public void goToPosition(double x, double y) {
    setTargetAnglesXY(x, y);
    while (!isArmMoving()) {
      loop();
    }
  }

  /**
   * Checks if the arm is currently moving by using the power of the motors.
   *
   * @return true if any of the motors are moving, false if they are not
   */
  public boolean isArmMoving() {
    boolean joint1m1, joint1m2, joint2;

    // true if it's moving
    joint1m1 = this.joint1m1.getPower() != 0;
    joint1m2 = this.joint1m2.getPower() != 0;
    joint2 = this.joint2.getPower() != 0;

    // return true if any of them are moving
    return joint1m1 || joint1m2 || joint2;
  }

  public void updateAngles() {
    previousJoint1Angle = currentJoint1Angle;
    previousJoint2Angle = currentJoint2Angle;
    previousTime = currentTime;
    currentTime = System.currentTimeMillis();
    currentJoint1Angle = ticksToRadians(joint1m1.getCurrentPosition());
    currentJoint2Angle = ticksToRadians(joint2.getCurrentPosition());
  }

  /**
   * Moves the arm to a certain position. x^2 + y^2 should not be > 4. It handles
   * that exception, but not well.
   *
   * @param x the horizontal position of the arm, max 2
   * @param y the wanted vertical position of the arm. -2 is the lowest position,
   *          2 is the highest position
   * @return the value of c <a href="https://i.imgur.com/zeBQZKn.png">in this
   *         drawing</a>
   */
  private double calculateC(double x, double y) {
    double answer = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    if (answer > 2) {
      return 2;
    } else {
      return answer;
    }
  }

  /**
   * calculates theta <a href="https://i.imgur.com/zeBQZKn.png">in this
   * drawing</a>
   *
   * @param c the value of c <a href="https://i.imgur.com/zeBQZKn.png">in this
   *          drawing</a>
   * @return the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in this
   *         drawing</a> in radians
   */
  private double calculateTheta(double c) {
    double theta = Math.acos(-(c * c / 2 - 1));
    return theta;
  }

  /**
   * calculates theta <a href="https://i.imgur.com/zeBQZKn.png">in this
   * drawing</a>.
   * magnitude of (x,y) <= 2
   *
   * @param x the wanted horizontal position of the arm, max 2
   * @param y the wanted vertical position of the arm, max 2
   * @return the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in this
   *         drawing</a> in radians
   */
  private double calculateTheta(double x, double y) {
    double c = calculateC(x, y);
    return calculateTheta(c);
  }

  /**
   * calculates phi <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>.
   * magnitude of (x,y) <= 2
   *
   * @param x the wanted horizontal position of the arm, max 2. should not be 0
   * @param y the wanted vertical position of the arm, max 2
   * @return the value of phi <a href="https://i.imgur.com/zeBQZKn.png">in this
   *         drawing</a> in radians between -pi/2 and pi/2
   *
   */
  private double calculatePhi(double x, double y) {
    if (x == 0) {
      x = 0.00000001;
    }
    double phi = Math.atan(y / x);
    return phi;
  }

  /**
   * calculates the angle of the first joint
   * <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   *
   * @param theta the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in
   *              this drawing</a>
   * @param phi   the value of phi <a href="https://i.imgur.com/zeBQZKn.png">in
   *              this drawing</a>
   * @return the angle of the first joint
   *         <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a> in
   *         radians (0 to pi)
   */
  private double calculateJoint1Angle(double theta, double phi) {
    double joint1Angle = Math.PI - (theta / 2) + phi;
    return joint1Angle;
  }

  /**
   * calculates the angle of the second joint
   * <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a>
   *
   * @param theta the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in
   *              this drawing</a>
   * @return the angle of the second joint
   *         <a href="https://i.imgur.com/zeBQZKn.png">in this drawing</a> in
   *         radians (0 to pi)
   */
  private double calculateJoint2Angle(double theta) {
    double joint2Angle = theta;
    return joint2Angle;
  }

  /**
   * calculates the angle of the claw aligner servo to make it perfectly
   * horizontal with the ground
   *
   * @param theta the value of theta <a href="https://i.imgur.com/zeBQZKn.png">in
   *              this drawing</a>
   * @param phi   the value of phi <a href="https://i.imgur.com/zeBQZKn.png">in
   *              this drawing</a>
   * @return the angle of the claw aligner servo to make it perfectly horizontal
   *         with the ground in radians
   */
  private double calculateClawAngle(double theta, double phi) {
    double clawAngle = phi - (Math.PI - theta / 2) + theta;
    return clawAngle;
  }

  private void setTargetJoint1Angle(double targetJoint1Angle) {
    this.targetJoint1Angle = targetJoint1Angle;
  }

  private void setTargetJoint2Angle(double targetJoint2Angle) {
    this.targetJoint2Angle = targetJoint2Angle;
  }

  private void setTargetAnglesThetaPhi(double theta, double phi) {
    setTargetJoint1Angle(calculateJoint1Angle(theta, phi));
    setTargetJoint2Angle(calculateJoint2Angle(theta));
  }

  public void setTargetAnglesXY(double x, double y) {
    setTargetAnglesThetaPhi(
        calculateTheta(calculateC(x, y)),
        calculatePhi(x, y));
  }

  public double getJoint1Power() {
    double error = targetJoint1Angle - currentJoint1Angle;
    return normalizeCorrection(error * JOINT1P);
  }

  public double getJoint2Power() {
    double error = targetJoint2Angle - currentJoint2Angle;
    return normalizeCorrection(error * JOINT2P);
  }

  private double normalizeCorrection(double correction) {
    if (correction > 1) {
      return 1;
    } else if (correction < -1) {
      return -1;
    } else {
      return correction;
    }
  }

  public double getCurrentTheta() {
    return currentJoint2Angle;
  }

  public double getCurrentPhi() {
    return currentJoint1Angle - (Math.PI - getCurrentTheta() / 2);
  }

  public double getTargetTheta() {
    return targetJoint2Angle;
  }

  public double getTargetPhi() {
    return targetJoint1Angle - (Math.PI - getTargetTheta() / 2);
  }

  private void setJoint1Power(double power) {
    joint1m1.setPower(power);
    joint1m2.setPower(-power);
  }

  private void setJoint2Power(double power) {
    joint2.setPower(power);
  }

  /**
   *
   * @param angle in radians between 0 and 5/3 pi
   */
  private void setClawAlignerTargetAngle(double angle) {
    clawAligner.setPosition(radiansToServoPosition(angle));
  }

  public double getTargetJoint1Angle() {
    return targetJoint1Angle;
  }

  public double getTargetJoint2Angle() {
    return targetJoint2Angle;
  }

  /**
   *
   * @param radians an angle between 0 and 5pi/3
   * @return a servo position between 0 and 1 inclusive
   */
  private double radiansToServoPosition(double radians) {
    return (radians * 3 / (5 * Math.PI));
  }

  private static double ticksToRadians(double ticks) {
    return 2 * Math.PI * ticks / 145.1;
  }

  private static double radiansToTicks(double radians) {
    return radians * 145.1 / (2 * Math.PI);
  }
}
