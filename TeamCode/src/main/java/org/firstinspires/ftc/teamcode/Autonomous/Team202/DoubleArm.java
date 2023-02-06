package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class DoubleArm extends Thread {

    public final double min_x = 0.2;
    public final double min_y = -1;

    private double tx, ty;

    private DcMotor right_motor, left_motor, joint2; // left_motor just follows right_motor
    private double right_motor_target = 0.0, joint_target = 0.0;
    private final double first_arm_zero = 410, second_arm_zero = -817, ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI;

    private Servo wrist;
    private double wrist_target = 0.0;

    private Telemetry telemetry;

    private boolean should_be_running = true;

    public void init(HardwareMap map, Telemetry t) {
        right_motor = map.get(DcMotor.class, "joint1right");
        left_motor = map.get(DcMotor.class, "joint1left");
        joint2 = map.get(DcMotor.class, "joint2");
        wrist = map.get(Servo.class, "clawAligner");

        telemetry = t;

        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        should_be_running = true;
    }

    public void set_position(double x, double y) {
        if (Math.abs(x * x + y * y) > 3.9) {
            return; // will throw an error
        }

        tx=x;
        ty=y;

        if (x < min_x) {
            set_position(min_x, y);
            return;
        }
        if (y < min_y) {
            set_position(x, min_y);
            return;
        }

        double magnitude = Math.sqrt(x * x + y * y);

        double temp_angle = Math.atan(y / x);
        double secondary_angle = Math.acos(magnitude / 2);
        double primary_angle = Math.PI - 2 * secondary_angle;

        double target_angle_one = temp_angle + secondary_angle;
        double target_angle_two = target_angle_one + primary_angle - Math.PI;
        double target_angle_three = 0 - target_angle_two;

        target_angle_one *= ticks_per_radian;
        target_angle_two *= ticks_per_radian;

        target_angle_three *= 180.0 / Math.PI; // angle

        target_angle_one += first_arm_zero;
        target_angle_two += second_arm_zero;
        target_angle_three = target_angle_three * wrist_m + wrist_b;

        right_motor_target = target_angle_one;

        joint_target = target_angle_two;

        wrist_target = target_angle_three;
    }

    public void set_height(double y) {
        set_position(1, y);
    }

    public void quit() {
        should_be_running = false;
    }

    public void run() {
        telemetry.addData("should be running", should_be_running);
        while (should_be_running) {
            right_motor.setPower(Math.max(Math.min(0.02 * (right_motor_target - right_motor.getCurrentPosition()), 0.4), -0.2));
            left_motor.setPower(right_motor.getPower());

            joint2.setPower(Math.max(Math.min(0.02 * (joint_target - joint2.getCurrentPosition()), 0.4), -0.2));

            wrist.setPosition(wrist_target);

            telemetry.addData("right motor", right_motor.getPower());
            telemetry.addData("left motor", left_motor.getPower());
            telemetry.addData("joint2", joint2.getPower());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.update();
        }
    }

    public boolean isBusy() {
        final double minPower = 0.005;
        return left_motor.getPower() > minPower && right_motor.getPower() > minPower || joint2.getPower() > minPower;
    }
}
