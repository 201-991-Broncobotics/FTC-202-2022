package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware {

  public HardwareMap map;
  public Telemetry telemetry;

  public BNO055IMU imu;

  public DcMotor RF = null;
  public DcMotor RB = null;
  public DcMotor LF = null;
  public DcMotor LB = null;

  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm =
      new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    RF = hardwareMap.get(DcMotor.class, "rightFront");
    RB = hardwareMap.get(DcMotor.class, "rightBack");
    LF = hardwareMap.get(DcMotor.class, "leftFront");
    LB = hardwareMap.get(DcMotor.class, "leftBack");

    RF.setDirection(DcMotor.Direction.REVERSE);
    RB.setDirection(DcMotor.Direction.REVERSE);
    LF.setDirection(DcMotor.Direction.FORWARD);
    LB.setDirection(DcMotor.Direction.FORWARD);

    RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //Rev2mDistanceSensor csensorTimeOfFlight = (Rev2mDistanceSensor)cubeSensor;
    //        Rev2mDistanceSensor dsensorTimeOfFlight = (Rev2mDistanceSensor)dSensor;

    this.map = hardwareMap;
  } //initializes everything

  public double getAngle() {
    Orientation angles = imu.getAngularOrientation(
      AxesReference.INTRINSIC,
      AxesOrder.YZX,
      AngleUnit.DEGREES
    );
    return angles.firstAngle;
  } //pretty obvious what it does

  public double getBatteryVoltage() {
    double result = Double.POSITIVE_INFINITY;
    for (VoltageSensor sensor : map.voltageSensor) {
      double voltage = sensor.getVoltage();
      if (voltage > 0) {
        result = Math.min(result, voltage);
      }
    }
    return result;
  } //we never need this

  public void ResetEncoders() {
    LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void DriveWithEncoders() {
    LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void DriveNormally() {
    LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void SetToEncoders() {
    LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void DriveDistance(double inches) {
    int Ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
    ResetEncoders();
    LF.setTargetPosition(Ticks);
    LB.setTargetPosition(Ticks);
    RF.setTargetPosition(Ticks);
    RB.setTargetPosition(Ticks);
    DriveWithEncoders();
  }

  public void DriveDistance(double inches, String Direction) {
    int Ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
    ResetEncoders();
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
    DriveWithEncoders();
  }

  public void turnDegree(int degrees) {
    double pastHeading = getAngle();
    double targetHeading = pastHeading + degrees;
    if (targetHeading > 360) {
      targetHeading -= 360;
    }
    if (targetHeading < 0) {
      targetHeading += 360;
    }
    if (getAngle() < targetHeading) {
      while (getAngle() < targetHeading) {
        LF.setPower(0.5);
        LB.setPower(0.5);
        RF.setPower(-0.5);
        RB.setPower(-0.5);
      }
    }
    if (getAngle() > targetHeading) {
      while (getAngle() > targetHeading) {
        LF.setPower(-0.5);
        LB.setPower(-0.5);
        RF.setPower(0.5);
        RB.setPower(0.5);
      }
    }
    SpeedSet(0);
  }

  public void turnWEncoders(int inches) {
    int Ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
    ResetEncoders();
    LF.setTargetPosition(-Ticks);
    LB.setTargetPosition(-Ticks);
    RF.setTargetPosition(Ticks);
    RB.setTargetPosition(Ticks);
    DriveWithEncoders();
  }

  public void turnEncoderDegree(int degrees) {
    double inchPer90 = 13.2;
    turnWEncoders((int) (((double) degrees / 90) * inchPer90));
  }

  public void SpeedSet(double speed) {
    LF.setPower(speed);
    LB.setPower(speed);
    RF.setPower(speed);
    RB.setPower(speed);
  }

  public boolean MotorsBusy() {
    return LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy();
  }
}