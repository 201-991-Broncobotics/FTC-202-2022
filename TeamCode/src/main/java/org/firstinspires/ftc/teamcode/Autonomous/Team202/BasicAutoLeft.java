package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;

@Autonomous(name="basic left")
@Disabled
public class BasicAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonLogic202 logic = new AutonLogic202();

        logic.init(hardwareMap, telemetry);

        waitForStart();

        logic.driveInches(24, DriveDirection.LEFT);

        sleep(1500);

        logic.stop();
    }
}
