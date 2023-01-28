package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;

@Autonomous(name="basic forward")
public class BasicAutonForward extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonLogic202 logic = new AutonLogic202();

        logic.init(hardwareMap, telemetry);

        waitForStart();

        logic.driveBad(DriveDirection.FORWARD);

        sleep(3000);

        logic.stop();

    }
}
