package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.robot_length;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.robot_width;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.AprilTag;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;

@Autonomous(name="202 apriltag auton")
public class Auton202AprilTags extends LinearOpMode {
    AutonLogic202 logic = new AutonLogic202();
    AprilTag aprilTag = new AprilTag();

    @Override
    public void runOpMode() {

        logic.init(hardwareMap, telemetry);
        aprilTag.init(hardwareMap);

        aprilTag.startScanning();

        waitForStart();

        // left of square to center of square
        logic.driveInches(12 - robot_width / 2, DriveDirection.RIGHT);
        sleep(2000);

        // stop right in front of the cone with sleeve
        logic.driveInches(36 - robot_length / 2 - CAMERA_FROM_CENTER, DriveDirection.FORWARD);
        sleep(2000);

        // drive to the center of the square
        logic.driveInches(CAMERA_FROM_CENTER, DriveDirection.FORWARD);
        sleep(2000);

        aprilTag.stopScanning();
        int id = aprilTag.getMostDetected();

        switch (id) {
            case tag1id: {
                logic.driveInches(24, DriveDirection.LEFT);
            }
            case tag2id: {

            }
            case tag3id: {
                logic.driveInches(24, DriveDirection.RIGHT);
            }
        }
    }
}