package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.robot_length;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.robot_width;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.AprilTag;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;

@Autonomous(name="202 april tag auto")
public class Auton202AprilTags extends LinearOpMode {
    AutonLogic202 logic = new AutonLogic202();
    AprilTag aprilTag = new AprilTag();

    @Override
    public void runOpMode() {

        logic.init(hardwareMap, telemetry);
        aprilTag.init(hardwareMap, telemetry);

        logic.closeClaw();
        aprilTag.startScanning();

        waitForStart();
        aprilTag.gameStarted();

        // put camera in front of april tag
        logic.driveInches(12 - robot_width / 2 - CAMERA_FROM_CENTER_X, DriveDirection.RIGHT);
        sleep(500);

        final double gap = 3;
        logic.driveInches(12 - robot_length / 2 - gap, DriveDirection.FORWARD);
        sleep(1000);

        logic.driveInches(CAMERA_FROM_CENTER_X, DriveDirection.RIGHT);
        sleep(3000);

        // get the camera closer to the april tag (might need to be removed)
        logic.driveInches(24 + gap, DriveDirection.FORWARD);
        sleep(500);

        // wait for the april tag to be scanned


        // drive to the center of the square that had the sleeve


        aprilTag.stopScanning();
        int id = aprilTag.getMostDetected();

        // move to the right square depending on april tag pos
        switch (id) {
            case tag1id: {
                logic.driveInches(24, DriveDirection.LEFT);
                break;
            }
            case tag2id: {
                break;
            }
            case tag3id: {
                logic.driveInches(24, DriveDirection.RIGHT);
                break;
            }
        }
    }
}