package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Systems.Utils.Arm;
import org.firstinspires.ftc.teamcode.Systems.Utils.DriveTrain;
import org.firstinspires.ftc.teamcode.Systems.Utils.PidMotor;

public class RobotHardware {

  public HardwareMap map;
  public Telemetry telemetry;

  public BNO055IMU imu;

  public DcMotor RF = null;
  public DcMotor RB = null;
  public DcMotor LF = null;
  public DcMotor LB = null;

  public DcMotor joint1m1, joint1m2, joint2 = null;
  public Servo clawAligner = null ;
  //  public Arm arm =  null;

  //  public PidMotor slide = null;

  public DriveTrain driveTrain = null;

  public void init(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm =
      new JustLoggingAccelerationIntegrator();

    RF = hardwareMap.get(DcMotor.class, "rightFront");
    RB = hardwareMap.get(DcMotor.class, "rightBack");
    LF = hardwareMap.get(DcMotor.class, "leftFront");
    LB = hardwareMap.get(DcMotor.class, "leftBack");

    joint1m1 = hardwareMap.get(DcMotor.class, "joint1m1");
    joint1m2 = hardwareMap.get(DcMotor.class, "joint1m2");
    joint2 = hardwareMap.get(DcMotor.class, "joint2");

    clawAligner = hardwareMap.get(Servo.class, "clawAligner");

    //    slide = new PidMotor(hardwareMap.get(DcMotor.class, "slide"), 1);

    driveTrain =
      new DriveTrain(
        RF,
        RB,
        LF,
        LB,
        0.4,
        0.2,
        0.1,
        0,
        hardwareMap.get(BNO055IMU.class, "imu")
      );

    driveTrain.imu.getAngularOrientation(
      AxesReference.INTRINSIC,
      AxesOrder.ZYX,
      AngleUnit.RADIANS
    );
    driveTrain.imu.initialize(parameters);

    RF.setDirection(DcMotor.Direction.REVERSE);
    RB.setDirection(DcMotor.Direction.REVERSE);
    LF.setDirection(DcMotor.Direction.FORWARD);
    LB.setDirection(DcMotor.Direction.FORWARD);

    driveTrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

    this.map = hardwareMap;
  }

  public double getAngle() {
    Orientation angles = imu.getAngularOrientation(
      AxesReference.INTRINSIC,
      AxesOrder.YZX,
      AngleUnit.DEGREES
    );
    return angles.firstAngle;
  } // pretty obvious what it does

  /**
   * @deprecated use {@link DriveTrain#stopAndResetEncoders()} instead
   */
  public void ResetEncoders() {
    driveTrain.stopAndResetEncoders();
  }

  /**
   * @deprecated use {@link DriveTrain#runToPosition()} instead
   */
  public void DriveWithEncoders() {
    driveTrain.runToPosition();
  }

  /**
   * @deprecated use {@link DriveTrain#runWithoutEncoders()} instead
   */
  public void DriveNormally() {
    driveTrain.runWithoutEncoders();
  }

  /**
   * @deprecated use {@link DriveTrain#runWithEncoders()} instead
   */
  public void SetToEncoders() {
    driveTrain.runWithEncoders();
  }

  /**
   * @deprecated use {@link DriveTrain#driveDistance(double)} instead
   */
  public void DriveDistance(double inches) {
    driveTrain.driveDistance(inches);
  }

  /**
   * @deprecated use {@link DriveTrain#driveDistance(double, String)} instead
   */
  public void DriveDistance(double inches, String Direction) {
    driveTrain.driveDistance(inches, Direction);
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
