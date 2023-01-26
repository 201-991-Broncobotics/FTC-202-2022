package org.firstinspires.ftc.teamcode.TeleOp.Team202;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp 202 no auton", group="Iterative Opmode")
public class TeleOp202NoAuton extends LinearOpMode {
    TeleOp202Logic logic = new TeleOp202Logic();
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        logic.resetEncoders();
        waitForStart();
        while (opModeIsActive()) {
            logic.tick(gamepad1, gamepad2);
            logic.execute_non_driver_controlled();
        }
    }
}