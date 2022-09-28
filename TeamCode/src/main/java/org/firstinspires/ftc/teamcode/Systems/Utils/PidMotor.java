package org.firstinspires.ftc.teamcode.Systems.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PidMotor {

  public DcMotor motor = null;
  private double p_weight, i_weight, d_weight = 0;

  private double current_time, previous_time, current_error, past_error, total_error =
    0;

  // just p
  public PidMotor(DcMotor motor, double p_weight) {
    this.motor = motor;
    this.p_weight = p_weight;
  }

  // p d
  public PidMotor(DcMotor motor, double p_weight, double d_weight) {
    this.motor = motor;
    this.p_weight = p_weight;
    this.d_weight = d_weight;
  }

  // pid
  public PidMotor(
    DcMotor motor,
    double p_weight,
    double i_weight,
    double d_weight
  ) {
    this.motor = motor;
    this.p_weight = p_weight;
    this.i_weight = i_weight;
    this.d_weight = d_weight;
  }

  /**
   * IMPORTANT: This method has an infinite while loop, so it WILL block the thread it is called on.
   * @param inches the inches you want the motor to go forward. Inches is based on omni wheel
   */
  public void setTargetInches(double inches) {
    int Ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
    setTarget(Ticks);
  }

  /**
   * IMPORTANT: This method has an infinite while loop, so it WILL block the thread it is called on.
   * @param target the position you want the motor to go to
   */
  public void setTarget(double target) {
    double correction;
    while (true) {
      correction = getCorrection(target);
      // could be a problem if correction never equals 0
      if (correction == 0) {
        break;
      }

      motor.setPower(Math.min(correction, 1));
    }
  }

  public double getCorrection(double target) {
    double correction = 0;
    double current_position = motor.getCurrentPosition();
    current_error = target - current_position;
    current_time = System.currentTimeMillis();
    double dt = current_time - previous_time;
    double de = current_error - past_error;
    double p = p_weight * current_error;
    double i = i_weight * total_error;
    double d = d_weight * (de / dt);
    correction = p + i + d;
    past_error = current_error;
    total_error += current_error;
    previous_time = current_time;
    return correction;
  }

  // helper methods
  private void ResetEncoder() {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  private void SetToEncoders() {
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
}
