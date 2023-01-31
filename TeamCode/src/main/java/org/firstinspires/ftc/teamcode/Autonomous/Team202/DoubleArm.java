package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import static org.firstinspires.ftc.teamcode.Robots.max_power;
import static org.firstinspires.ftc.teamcode.Robots.min_power;
import static org.firstinspires.ftc.teamcode.Robots.p_weights;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.first_arm_zero;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.second_arm_zero;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.ticks_per_radian;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DoubleArm extends Thread {

    private static DcMotor right_motor, left_motor, joint2; // left_motor just follows right_motor
    private static double right_motor_target = 0.0, joint_target = 0.0;

    private static Servo wrist, claw;
    private static double wrist_target = 0.0;

    private static double claw_target = claw_closed;

    private static boolean should_be_running = true;

    public void init(HardwareMap map) {
        right_motor = map.get(DcMotor.class, "joint1right");
        left_motor = map.get(DcMotor.class, "joint1left");
        joint2 = map.get(DcMotor.class, "joint2");
        wrist = map.get(Servo.class, "clawAligner");
        claw = map.get(Servo.class, "claw");
        claw.setPosition(claw_closed);

        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public static void open_claw() {
        claw_target = claw_open;
    }

    public static void close_claw() {
        claw_target = claw_closed;
    }

    public static void set_position(double x, double y) {
        if (Math.abs(x * x + y * y) > 4) {
            return; // will throw an error
        }
        double magnitude = Math.sqrt(x * x + y * y);

        double temp_angle = Math.atan(y / x);
        double secondary_angle = Math.acos(magnitude / 2);
        double primary_angle = Math.PI - 2 * secondary_angle;

        double target_angle_one = temp_angle + secondary_angle;
        double target_angle_two = target_angle_one + primary_angle - Math.PI;
        double target_angle_three = 0 - target_angle_two;
        // removing the initial angle

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
        while (should_be_running) {
            right_motor.setPower(
                    Math.max(Math.min(p_weights[0] * (right_motor_target - right_motor.getCurrentPosition()), max_power[0]),
                            min_power[0]));
            left_motor.setPower(right_motor.getPower());

            joint2.setPower(Math.max(Math.min(p_weights[1] * (joint_target - joint2.getCurrentPosition()), max_power[1]),
                    min_power[1]));

            wrist.setPosition(wrist_target);
            claw.setPosition(claw_target);
        }
    }
}
