package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogicBase;

public class AutonLogic202 {
    public static DoubleArm arm;
    public static Servo claw;
    private static HardwareMap map;
    private static DcMotor rf, lf, rb, lb;

    private static DcMotor[] wheel_list = new DcMotor[4];

    public static void init(HardwareMap _map, Telemetry _telemetry) {

        map = _map;
        arm = new DoubleArm();
        claw = map.get(Servo.class, "claw");

        rf = map.get(DcMotor.class, "rightFront");
        lf = map.get(DcMotor.class, "leftFront");
        rb = map.get(DcMotor.class, "rightBack");
        lb = map.get(DcMotor.class, "leftBack");
        wheel_list = new DcMotor[] {
                map.get(DcMotor.class, "rightFront"),
                map.get(DcMotor.class, "rightBack"),
                map.get(DcMotor.class, "leftBack"),
                map.get(DcMotor.class, "leftFront"),

        };
        wheel_list[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel_list[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel_list[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel_list[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        lf.setDirection(DcMotorSimple.Direction.REVERSE);
//        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initArm() {
        arm.init(map);
        arm.start();
    }

    private static void resetEncoders() {
        wheel_list[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_list[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_list[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_list[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheel_list[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel_list[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel_list[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel_list[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static void driveWithEncoders() {

        wheel_list[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel_list[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel_list[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel_list[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = 0.4;

        for (int i = 0; i < 4; i++) {
            wheel_list[i].setPower(
                (wheel_list[i].getTargetPosition() < wheel_list[i].getCurrentPosition()) ? -power : power);
        }

        while (wheel_list[0].isBusy() || wheel_list[1].isBusy() || wheel_list[2].isBusy() || wheel_list[3].isBusy()) {
            // wait
        }

        stop();
    }

    public static void setWheelPositions(double[] positions) {
        if (positions.length != 4) {
            throw new IllegalArgumentException("positions len != 4, it is " + positions.length);
        }
        int index = 0;
        for (double position : positions) {
            wheel_list[index].setTargetPosition((int) position);
            index++;
        }

    }

    public static void driveBad(DriveDirection direction) {
        switch (direction) {
            case FORWARD: {
                drive(new double[] { 0, 1 }, 0);
                break;
            }
            case LEFT: {
                drive(new double[] { -1, 0 }, 0);
                break;
            }
            case RIGHT: {
                drive(new double[] { 1, 0 }, 0);
                break;
            }
            case BACKWARD: {
                drive(new double[] { 0, -1 }, 0);
            }
        }
    }

    public static void stop() {
        wheel_list[0].setPower(0);
        wheel_list[1].setPower(0);
        wheel_list[2].setPower(0);
        wheel_list[3].setPower(0);
    }

    public static void driveTicks(double ticks, DriveDirection direction) {
        resetEncoders();
        switch (direction) {
            case FORWARD: {
                setWheelPositions(new double[] { ticks, ticks, ticks, ticks });
                break;
            }
            case BACKWARD: {
                setWheelPositions(new double[] { -ticks, -ticks, -ticks, -ticks });
                break;
            }
            case LEFT: {
                setWheelPositions(new double[] { +ticks, -ticks, +ticks, -ticks });
                break;
            }
            case RIGHT: {
                setWheelPositions(new double[] { -ticks, +ticks, -ticks, +ticks });
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

    public static void drive(double[] vec, double turnAmount) {

        if (vec.length != 2) {
            throw new IllegalArgumentException("drive function vec len > 2");
        }

        double x = vec[0];
        double y = vec[1];

        double[] powers = new double[4];

        // rightFront
        powers[0] = y - x;
        // rightBack
        powers[1] = y + x;
        // leftBack
        powers[2] = y - x;
        // leftFront
        powers[3] = y + x;

        powers[0] -= turnAmount;
        powers[1] -= turnAmount;
        powers[2] += turnAmount;
        powers[3] += turnAmount;

        if (max(powers) > 1) {
            double max = max(powers);
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }

        for (int i = 0; i < 4; i++)
            wheel_list[i].setPower(powers[i]);
    }

    /**
     * finds the greatest absolute val in the arr
     */
    private static double max(double[] arr) {
        double max = 1;
        for (double v : arr) {
            max = Math.max(max, Math.abs(v));
        }

        return max;
    }

    public static void openClaw() {
        claw.setPosition(claw_open);
    }

    public static void closeClaw() {
        claw.setPosition(claw_closed);
    }
}
