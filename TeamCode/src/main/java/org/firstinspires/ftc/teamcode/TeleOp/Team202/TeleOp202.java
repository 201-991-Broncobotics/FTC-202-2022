package org.firstinspires.ftc.teamcode.TeleOp.Team202;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;
import static org.firstinspires.ftc.teamcode.Robots.*;

class TeleOp202Logic extends TeleOpLogicBase {

    public static double starting_time;

    public static final double lift_speed = 1.3, in_speed = 1.3, stick_lift_speed = 2, stick_in_speed = 2;
    public static double tx = 1, ty = 0;

    public final static double
            first_arm_zero =     410,
            second_arm_zero =    -817;

    public static final double ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI; // 2786.2109868741 ticks per revolution

    public static DcMotor left_motor = null;

    public static Servo claw;

    public static boolean wristRotated = false;

    public static void execute_non_driver_controlled() {
        if (buttons[4]) ty += delta_time * lift_speed;
        if (buttons[5]) ty -= delta_time * lift_speed;
        if (buttons[6]) tx -= delta_time * in_speed;
        if (buttons[7]) tx += delta_time * in_speed;

        tx += axes[0] * delta_time * stick_in_speed; // left stick x
        tx += axes[1] * delta_time * stick_in_speed * 1.0/3.0; // right stick x
        ty += axes[2] * delta_time * stick_lift_speed; // left stick y
        ty += axes[3] * delta_time * stick_lift_speed * 1.0/3.0; // right stick y

        if (buttons[8]) {
            // op left bumper
            claw.setPosition(0.75);
        }
        if (buttons[9]) {
            // op right bumper
            claw.setPosition(0.35);
        }

        if (buttons[0]) /* op a */ {
            ty = -1;
            tx = 1;
        }
        if (buttons[1]) /* op b */ {
            ty = 1.25;
            tx = 0.5;
        }
        if (buttons[2]) /* op x */ {
            ty = -0.7;
            tx = 0.3;
        }
        if (buttons[3]) /* op y */ {
            ty = 1.95;
            tx = Math.sqrt(4 - ty * ty);
        }

        if (axes[4] > 0.5) /* op left trigger */ {
            wristRotated = false;
        }
        if (axes[5] > 0.5) /* op right trigger */ {
            wristRotated = true;
        }


        if (ty >= 2) ty = 2;
        if (tx >= 2) tx = 2;
        if (ty < -1) ty = -1;
        if (tx < 0.7) {
            if (ty < -0.3) {
                tx = 0.7;
            } else if (tx < 0.3) tx = 0.3;
        }
        double magnitude = Math.sqrt(tx * tx + ty * ty);

        if (magnitude > 2) {
            double ratio =  1.995 / magnitude;
            tx *= ratio;
            ty *= ratio;
            magnitude = Math.sqrt(tx * tx + ty * ty);
        }

        double temp_angle = Math.atan(ty / tx);
        double secondary_angle = Math.acos(magnitude / 2);
        double primary_angle = Math.PI - 2 * secondary_angle;

        double target_angle_one = temp_angle + secondary_angle;
        double target_angle_two = target_angle_one + primary_angle - Math.PI;
        double target_angle_three = 0 - target_angle_two - (wristRotated ? Math.PI / 2 : 0);
        // removing the initial angle

        target_angle_one *= ticks_per_radian;
        target_angle_two *= ticks_per_radian;

        target_angle_three *= 180.0 / Math.PI; // angle
            // 300 is the maximum angle --> 5/6 pi --> sets target angle to 1

        target_angle_one += first_arm_zero;
        target_angle_two += second_arm_zero;
        target_angle_three = target_angle_three * 0.00334 + 0.33577; // very good guessing ig

        dc_target_positions[0] = target_angle_one;
        left_motor.setPower(dc_motor_list[0].getPower());

        dc_target_positions[1] = target_angle_two;

        servo_target_positions[0] = target_angle_three;
        telemetry.addData("position", servo_list[0].getPosition());

        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);

        telemetry.update();
    }

    //Initialization

    public static void init(HardwareMap hm, Telemetry tm) {
        starting_time = System.nanoTime() / 1000000000.0;
        init202();
        initialize_logic(hm, tm);
        setZeroAngle(0);
        setKeybinds();
        set_button_types();
        left_motor = map.get(DcMotor.class, "joint1left");
        DcMotor joint2 = map.get(DcMotor.class, "joint2");
        claw = map.get(Servo.class, "claw");
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo_target_positions[0] = 0.6;
    }

    public static void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 0, localizer);
    }

    public static void setKeybinds() {

        //new_keybind("clawAligner", "operator right_stick_y", "default", 0.3, 0.3);
        //new_keybind("claw", "operator right_trigger", "default", 0.66, 0.66);
        //new_keybind("claw", "operator dpad_left", "default", "gradient", -0.66);

        // TODO: Figure out why operator left_trigger isn't working for this >:(

    }
}

@TeleOp(name="TeleOp 202", group="Iterative Opmode")
public class TeleOp202 extends LinearOpMode {
    TeleOp202Logic logic = new TeleOp202Logic();
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            logic.tick(gamepad1, gamepad2);
            logic.execute_non_driver_controlled();
        }
    }
}