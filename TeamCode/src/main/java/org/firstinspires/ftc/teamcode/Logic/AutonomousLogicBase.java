package org.firstinspires.ftc.teamcode.Logic;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutonomousLogicBase extends RobotHardware {

    public static void wait(double seconds) {
        double start = System.nanoTime() / 1000000000.0;
        while (System.nanoTime() / 1000000000.0 - start < seconds) { /* wait */ }
    }

    public static void stop() {
        for (DcMotor motor : dc_motor_list) {
            motor.setPower(0);
        }
        for (DcMotor motor : wheel_list) {
            motor.setPower(0);
        }
        //alternatives: stop() in LinearOpMode or throw an error
    }

    public static void driveForward(double power) {
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower(power);
    }

    public static void strafe(double power) {
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower((i % 2 == 0 ? 1 : -1) * power);
    }

    public static void turn(double power) {
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower((i > 1 ? -1 : 1) * power);
    }

    public static void setTargetPositions(double[] positions) {
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setTargetPosition((int) positions[i]);
    }

    public static void resetDriveEncoders() {
        for (int i = 0; i < wheel_list.length; i++) {
            wheel_list[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel_list[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public static void turnDegree(double degrees) {
        double startingAngle = getAngle();

        double targetHeading = startingAngle + degrees;
        while (targetHeading < 0 || targetHeading >= 360) {
            targetHeading += 360 * (targetHeading < 0 ? 1 : -1);
        }

        int factor = (((0 < targetHeading - startingAngle) && (targetHeading - startingAngle < 180)) || (targetHeading - startingAngle < -180) ? 1 : -1);
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower((i < 2 ? -0.5 : 0.5) * factor);

        while ((getAngle() - targetHeading) * (getAngle() - targetHeading) < 25) { /* wait */ }
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower(0);
    }

    public static void driveTicks(double ticks, DriveDirection direction) {
        resetDriveEncoders();
        switch (direction) {
            case FORWARD: {
                setTargetPositions(new double[]{ticks, ticks, ticks, ticks});
                break;
            }
            case BACKWARD: {
                setTargetPositions(new double[]{-ticks, -ticks, -ticks, -ticks});
                break;
            }
            case LEFT: {
                setTargetPositions(new double[]{-ticks, ticks, ticks, -ticks});
                break;
            }
            case RIGHT: {
                setTargetPositions(new double[]{ticks, -ticks, -ticks, ticks});
                break;
            }
        }
        driveWithEncoders();
    }

    public static void driveTicks(double ticks) {
        driveTicks(ticks, DriveDirection.FORWARD);
    }

    public static void driveInches(double inches, DriveDirection direction) {
        int ticks = (int) Math.round(inches / ((3 * Math.PI) / 767));
        driveTicks(ticks, direction);
    }

    public static void driveInches(double inches) {
        driveInches(inches, DriveDirection.FORWARD);
    }

    //DC Motors
    public static void setPower(String name, double power) {
        dc_motor_list[dc_motor_names.indexOf(name)].setPower(power);
    }

    public static void resetEncoder(String name) {
        dc_motor_list[dc_motor_names.indexOf(name)].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dc_motor_list[dc_motor_names.indexOf(name)].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Servos
    public static void setPosition(String name, double position) {
        servo_list[servo_names.indexOf(name)].setPosition(position);
    }

    public static void driveWithEncoders(){
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void initialize_RoadRunner() {}

    public static void initialize_tensorflow() {}
}
