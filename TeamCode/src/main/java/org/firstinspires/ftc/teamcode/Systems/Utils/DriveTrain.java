package org.firstinspires.ftc.teamcode.Systems.Utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {

  DcMotor RF, RB, LF, LB;

  double p_weight, d_weight;

  double p_angle_weight, d_angle_weight;

  public BNO055IMU imu;

  double previous_angle_error, previous_angle_time, current_angle_error, current_angle_time;
  double previous_drive_error, previous_drive_time, current_drive_error, current_drive_time;

  public DriveTrain(
    DcMotor RF,
    DcMotor RB,
    DcMotor LF,
    DcMotor LB,
    double p_weight,
    double d_weight,
    double p_angle_weight,
    double d_angle_weight,
    BNO055IMU imu
  ) {
    this.RF = RF;
    this.RB = RB;
    this.LF = LF;
    this.LB = LB;
    this.p_weight = p_weight;
    this.d_weight = d_weight;
    this.p_angle_weight = p_angle_weight;
    this.d_angle_weight = d_angle_weight;
    this.imu = imu;
  }

  /**
   * Drives forward using PID while correcting angles using PID to keep it straight
   * @param ticks how far you want the robot to go
   */
  public void driveForward(double ticks) {
    stopAndResetEncoders();
    double startAngle = imu.getAngularOrientation().firstAngle;
    Double[] startEncoders = new Double[] {
      (double) RF.getCurrentPosition(),
      (double) RB.getCurrentPosition(),
      (double) LF.getCurrentPosition(),
      (double) LB.getCurrentPosition(),
    };
  }

  private double getAngleCorrection(double targetAngle) {
    current_angle_error = targetAngle - imu.getAngularOrientation().firstAngle;

    double angleCorrection =
      p_angle_weight *
      current_angle_error +
      d_angle_weight *
      (current_angle_error - previous_angle_error) /
      (current_angle_time - previous_angle_time);

    previous_angle_error = current_angle_error;
    previous_angle_time = current_angle_time;

    return angleCorrection;
  }

  private double getDriveCorrection(double target) {
    current_drive_time = System.currentTimeMillis();
    current_drive_error = target - RF.getCurrentPosition();

    double driveCorrection =
      p_weight *
      current_drive_error +
      d_weight *
      (current_drive_error - previous_drive_error) /
      (current_drive_time - previous_drive_time);

    previous_drive_error = current_drive_error;
    previous_drive_time = current_drive_time;

    return driveCorrection;
  }

  public void driveDistance(double inches) {
    int Ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
    stopAndResetEncoders();
    LF.setTargetPosition(Ticks);
    LB.setTargetPosition(Ticks);
    RF.setTargetPosition(Ticks);
    RB.setTargetPosition(Ticks);
    runToPosition();
  }

  public void driveDistance(double inches, String Direction) {
    int Ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
    stopAndResetEncoders();
    switch (Direction) {
      case "Right":
        LF.setTargetPosition(-Ticks);
        LB.setTargetPosition(Ticks);
        RF.setTargetPosition(Ticks);
        RB.setTargetPosition(-Ticks);
        break;
      case "Left":
        LF.setTargetPosition(Ticks);
        LB.setTargetPosition(-Ticks);
        RF.setTargetPosition(-Ticks);
        RB.setTargetPosition(Ticks);
        break;
      case "Backward":
        LF.setTargetPosition(Ticks);
        LB.setTargetPosition(Ticks);
        RF.setTargetPosition(Ticks);
        RB.setTargetPosition(Ticks);
        break;
      case "Forward":
        LF.setTargetPosition(-Ticks);
        LB.setTargetPosition(-Ticks);
        RF.setTargetPosition(-Ticks);
        RB.setTargetPosition(-Ticks);
        break;
    }
    runToPosition();
  }

  public void turnWEncoders(int inches) {
    int Ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
    stopAndResetEncoders();
    LF.setTargetPosition(-Ticks);
    LB.setTargetPosition(-Ticks);
    RF.setTargetPosition(Ticks);
    RB.setTargetPosition(Ticks);
    runToPosition();
  }

  public void turnEncoderDegree(int degrees) {
    double inchPer90 = 13.2;
    turnWEncoders((int) (((double) degrees / 90) * inchPer90));
  }

  public void setSpeed(double speed) {
    LF.setPower(speed);
    LB.setPower(speed);
    RF.setPower(speed);
    RB.setPower(speed);
  }

  public boolean MotorsBusy() {
    return LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy();
  }

  public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zpb) {
    RF.setZeroPowerBehavior(zpb);
    RB.setZeroPowerBehavior(zpb);
    LF.setZeroPowerBehavior(zpb);
    LB.setZeroPowerBehavior(zpb);
  }

  public void stopAndResetEncoders() {
    LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void runToPosition() {
    LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void runWithoutEncoders() {
    LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void runWithEncoders() {
    LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
}
