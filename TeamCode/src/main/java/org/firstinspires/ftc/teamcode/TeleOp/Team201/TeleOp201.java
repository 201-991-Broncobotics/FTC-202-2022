package org.firstinspires.ftc.teamcode.TeleOp.Team201;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;
import static org.firstinspires.ftc.teamcode.Robots.*;

class TeleOp201Logic extends TeleOpLogicBase {

    DcMotor rightMotor;
    Servo claw;
    double t_position = 0.6;

    public void execute_non_driver_controlled() {

        telemetry.addData("Angle?", getAngle());
        telemetry.addData("Angle V2", current_angle = 0 - getAngle() - zero_angle); //Only different value if not starting robot straight ahead
        //Positive = Rotated clockwise

        telemetry.addData("Power", dc_motor_list[0].getPower());
        telemetry.addData("Servo Power?", cr_servo_list[0].getPower());

        telemetry.update();

        rightMotor.setPower(dc_motor_list[0].getPower());

        if (buttons[keys.indexOf("driver a")]) {
            t_position = 0.6;
        } else if (buttons[keys.indexOf("driver b")]) {
            t_position = 0.2;
        }

        claw.setPosition(t_position);

        telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init(HardwareMap hm, Telemetry tm) {
        init201();
        initialize_logic(hm, tm);
        setZeroAngle(0);
        set_keybinds();
        set_button_types();
        rightMotor = map.get(DcMotor.class, "Right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        claw = map.get(Servo.class, "Scissor");

    }

    public void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 0, localizer);
    }

    public void set_keybinds() {
        // Arm
        new_keybind("Left", "operator dpad_up", "default", "normal", 0.9);
        new_keybind("Left", "operator dpad_down", "default", "normal", 0.1);

        // V4B
        new_keybind("Virtual", "operator right_stick_y", "default", 1.0, 1.0);
    }
}

@TeleOp(name="TeleOp 201", group="Iterative Opmode")
@Disabled
public class TeleOp201 extends LinearOpMode {
    TeleOp201Logic logic = new TeleOp201Logic();
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        waitForStart();
        if (useRoadRunner) {
            logic.initRoadRunner(new StandardTrackingWheelLocalizer(hardwareMap, logic));
        }
        while (opModeIsActive()) {
            logic.tick(gamepad1, gamepad2);
            logic.execute_non_driver_controlled();
        }
    }
}