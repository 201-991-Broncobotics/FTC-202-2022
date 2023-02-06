package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;

public class AutonLogic202 {
    public DoubleArm arm;
    public static Servo claw;
    private static HardwareMap map;
    private static DcMotor rf, lf, rb, lb;

    private static DcMotor[] wheel_list = new DcMotor[4];
    private static Telemetry telem;

    public void init(HardwareMap _map, Telemetry _telemetry) {

        telem = _telemetry;

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

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initArm() {
        arm.init(map, telem);
        arm.start();
    }

    private void resetEncoders() {
        wheel_list[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_list[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_list[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_list[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheel_list[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel_list[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel_list[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel_list[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveWithEncoders() {

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

    public void setWheelPositions(double[] positions) {
        if (positions.length != 4) {
            throw new IllegalArgumentException("positions len != 4, it is " + positions.length);
        }
        int index = 0;
        for (double position : positions) {
            wheel_list[index].setTargetPosition((int) position);
            index++;
        }

    }

    public void stop() {
        wheel_list[0].setPower(0);
        wheel_list[1].setPower(0);
        wheel_list[2].setPower(0);
        wheel_list[3].setPower(0);
    }

    public void driveTicks(double ticks, DriveDirection direction) {
        resetEncoders();
        // dont worry about why its all backwards :)
        switch (direction) {
            case BACKWARD: {
                setWheelPositions(new double[] { +ticks, +ticks, +ticks, +ticks });
                break;
            }
            case FORWARD: {
                setWheelPositions(new double[] { -ticks, -ticks, -ticks, -ticks });
                break;
            }
            case LEFT: {
                setWheelPositions(new double[] { +ticks * EXTRA_STRAFE_TICKS, -ticks * EXTRA_STRAFE_TICKS, +ticks * EXTRA_STRAFE_TICKS, -ticks * EXTRA_STRAFE_TICKS });
                break;
            }
            case RIGHT: {
                setWheelPositions(new double[] { -ticks * EXTRA_STRAFE_TICKS, +ticks * EXTRA_STRAFE_TICKS, -ticks * EXTRA_STRAFE_TICKS, +ticks * EXTRA_STRAFE_TICKS});
                break;
            }
        }
        driveWithEncoders();
    }

    public void driveTicks(double ticks) {
        driveTicks(ticks, DriveDirection.FORWARD);
    }

    public void driveInches(double inches, DriveDirection direction) {
        int ticks = (int) (inches * DRIVETRAIN_TICKS_PER_INCH);
        driveTicks(ticks, direction);
    }

    public void driveInches(double inches) {
        driveInches(inches, DriveDirection.FORWARD);
    }

    public void openClaw() {
        claw.setPosition(claw_open);
    }

    public void closeClaw() {
        claw.setPosition(claw_closed);
    }

    public void setArmPos(double x, double y) {
        arm.set_position(x,y);
    }

    public void waitForArm() {
        int runCount = 0;
        while (arm.isBusy()) {
            telem.addData("arm run count", runCount);
        }
    }
}
