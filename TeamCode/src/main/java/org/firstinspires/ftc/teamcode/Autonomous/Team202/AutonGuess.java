package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Robots.*;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.OpenCV;

@Autonomous(name = "Autonomous 202 GUESS")
@Disabled
public class AutonGuess extends LinearOpMode {
    String qrcodeResult;

    final double startX = 24 + robot_width / 2;
    final double startY = robot_length / 2;

    @Override
    public void runOpMode() throws InterruptedException {
        AutonLogic202 logic = new AutonLogic202();

        logic.init(hardwareMap, telemetry);

        waitForStart();

        logic.driveInches(48 - robot_length/2);
        sleep(2000);
        logic.driveInches(12 - robot_width / 2, DriveDirection.RIGHT);
        sleep(2000);



    }
}