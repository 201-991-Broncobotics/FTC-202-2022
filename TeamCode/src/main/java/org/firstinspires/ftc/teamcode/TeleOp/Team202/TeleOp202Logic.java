package org.firstinspires.ftc.teamcode.TeleOp.Team202;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;

import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;

class TeleOp202Logic extends TeleOpLogicBase {

    public static double starting_time;

    public static double tx = 1, ty = 0;

    public static DcMotor left_motor, joint2;

    public static Servo claw;

    public static double wristAngle = 0;

    public static void execute_non_driver_controlled() {
        if (buttons[4]) ty += delta_time * lift_speed;
        if (buttons[5]) ty -= delta_time * lift_speed;
        if (buttons[6]) tx += delta_time * in_speed;
        if (buttons[7]) tx -= delta_time * in_speed;

        tx += axes[0] * delta_time * stick_in_speed; // left stick x
        tx += axes[1] * delta_time * stick_in_speed * 1.0 / 3.0; // right stick x
        ty += axes[2] * delta_time * stick_lift_speed; // left stick y
        ty += axes[3] * delta_time * stick_lift_speed * 1.0 / 3.0; // right stick y

        if (buttons[8]) {
            // op left bumper
            claw.setPosition(claw_open);
        }
        if (buttons[9]) {
            // op right bumper
            claw.setPosition(claw_closed);
        }

        if (buttons[0]) /* op a */ {
            ty = -0.9;
            tx = 1;
        }
        if (buttons[1]) /* op b */ {
            ty = 1.25;
            tx = 0.5;
        }
        if (buttons[2]) /* op x */ {
            ty = -0.6;
            tx = 0.3;
        }
        if (buttons[3]) /* op y */ {
            ty = 1.93;
            tx = Math.sqrt(4 - ty * ty);
        }

        double lt = axes[4];
        double rt = axes[5];

        if (lt > 0.5 && rt > 0.5) {
            // reset wrist angle to even with ground.
            wristAngle = 0;
        } else {
            wristAngle += (rt - lt) * delta_time * wrist_angle_speed;
        }

        if (ty >= 2) ty = 2;
        if (tx >= 2) tx = 2;
        if (ty < -1.2) ty = -1.2;
        if (tx < 0.7) {
            if (ty < -0.65) {
                tx = 0.7;
            } else if (tx < 0.3) {
                tx = 0.3;
            }
        }
        double magnitude = Math.sqrt(tx * tx + ty * ty);

        if (magnitude > 2) {
            double ratio = 1.995 / magnitude;
            tx *= ratio;
            ty *= ratio;
            magnitude = Math.sqrt(tx * tx + ty * ty);
        }

        double temp_angle = Math.atan(ty / tx);
        double secondary_angle = Math.acos(magnitude / 2);
        double primary_angle = Math.PI - 2 * secondary_angle;

        double target_angle_one = temp_angle + secondary_angle;
        double target_angle_two = target_angle_one + primary_angle - Math.PI;
        double target_angle_three = 0 - target_angle_two - wristAngle;
        // removing the initial angle

        target_angle_one *= ticks_per_radian;
        target_angle_two *= ticks_per_radian;

        target_angle_three *= 180.0 / Math.PI; // angle

        target_angle_one += first_arm_zero;
        target_angle_two += second_arm_zero;
        target_angle_three = target_angle_three * wrist_m + wrist_b;

        dc_target_positions[0] = target_angle_one;
        left_motor.setPower(dc_motor_list[0].getPower());

        dc_target_positions[1] = target_angle_two;

        servo_target_positions[0] = target_angle_three;
        telemetry.addData("position", servo_list[0].getPosition());

        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);

        telemetry.update();
    }

    public static void init(HardwareMap hm, Telemetry tm) {
        starting_time = System.nanoTime() / 1000000000.0;
        init202();
        initialize_logic(hm, tm);
        setZeroAngle(0);
        setKeybinds();
        set_button_types();
        left_motor = map.get(DcMotor.class, "joint1left");
        joint2 = map.get(DcMotor.class, "joint2");
        claw = map.get(Servo.class, "claw");
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(claw_closed);
    }

    public static void resetEncoders() {
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void setKeybinds() {
    }
}
