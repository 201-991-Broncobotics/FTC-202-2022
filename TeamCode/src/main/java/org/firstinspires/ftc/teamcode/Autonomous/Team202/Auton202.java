package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Robots.*;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.claw_closed;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.OpenCV;

@Autonomous(name = "Autonomous 202 Main")
public class Auton202 extends LinearOpMode {
    String qrcodeResult;

    final double startX = 24 + robot_width / 2;
    final double startY = robot_length / 2;

    @Override
    public void runOpMode() throws InterruptedException {
        AutonLogic202 logic = new AutonLogic202();

        logic.init(hardwareMap, telemetry);

        logic.claw.setPosition(claw_closed);

        OpenCV camera = new OpenCV(hardwareMap, telemetry, "Webcam 1");

        camera.start();

        waitForStart();

        double distance = 36 - robot_length / 2 - 24;

        logic.driveInches(4 * distance / 5 , DriveDirection.BACKWARD);
        sleep(2000);

        camera.lock();
        logic.driveInches(distance / 5, DriveDirection.BACKWARD);

        String result = camera.getResult();
        camera.stopStreaming();

        if (result.equals("None")) result = "Cyan";

        telemetry.addData("Detected color", result);
        telemetry.update();

        switch (result) {
            // 1
            case "Purple": {
                logic.driveInches(2, DriveDirection.BACKWARD);
                sleep(2000);
                // it went backwards idk
                logic.driveInches(18, DriveDirection.LEFT);
                break;
            }

            // 2
            case "Yellow": {
                logic.driveInches(2, DriveDirection.BACKWARD);
                sleep(2000);
                break;
            }

            // 3
            case "Cyan": {
                logic.driveInches(2, DriveDirection.BACKWARD);
                sleep(2000);
                logic.driveInches(18, DriveDirection.RIGHT);
                break;
            }
        }

        camera.stopStreaming();
    }
}