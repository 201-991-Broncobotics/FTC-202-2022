package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;

@Autonomous(name="test right")
@Disabled
public class TestRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutonLogic202 logic = new AutonLogic202();
        logic.init(hardwareMap, telemetry);

        waitForStart();

        logic.driveInches(24, DriveDirection.RIGHT);

        sleep(1000);
    }
}
