package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.*;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.robot_length;
import static org.firstinspires.ftc.teamcode.TeleOp.Team202.Constants.robot_width;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.AprilTag;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.DriveDirection;

@Autonomous(name="Auton 202")
public class Auton202 extends LinearOpMode {
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

        sleep(1000);
        final double gap = 5.0;
        final double xgap = 2.5;
        logic.driveInches(gap, DriveDirection.FORWARD);

        // put camera in front of april tag
        logic.driveInches(12 - robot_width / 2 - CAMERA_FROM_CENTER_X, DriveDirection.RIGHT);
        sleep(500);

        // wait for april tags
        sleep(3000);

        aprilTag.stopScanning();
        int id = aprilTag.getMostDetected();

        // drive to the center of the square that had the sleeve
        logic.driveInches(24 - CAMERA_FROM_CENTER_X - xgap, DriveDirection.LEFT);
        sleep(500);

        logic.driveInches(48 - gap + robot_length / 2, DriveDirection.FORWARD);
        sleep(500);

        logic.initArm();
        // wait for arm to settle
        sleep(500);
        final double armX = 0.3;

        logic.setArmPos(0.3,1.95);
        sleep(4000);

        // the cone will be ARM_FROM_CENTER + Math.sqrt(4 - 1.93 * 1.93 - .01) * ARM_LENGTH + CLAW_OFFSET
        // from the center of the robot, so we need to adjust for that.

        // ~11
        final double CONE_FROM_CENTER = ARM_FROM_CENTER + armX * ARM_LENGTH + CLAW_OFFSET;

        logic.driveInches(12 - CONE_FROM_CENTER, DriveDirection.FORWARD);
        sleep(1000);

        logic.driveInches(36 - xgap, DriveDirection.RIGHT);
        sleep(1000);

        logic.openClaw();

        logic.setArmPos(1, 0);
        logic.arm.quit();

        sleep(1000);

        logic.openClaw();

        logic.driveInches(CONE_FROM_CENTER - 12, DriveDirection.FORWARD);
        sleep(500);

        switch (id) {
            case tag1id: {
                logic.driveInches(36,DriveDirection.LEFT);
                break;
            }
            case tag2id: {
                logic.driveInches(12, DriveDirection.LEFT);
                break;
            }
            case tag3id: {
                logic.driveInches(12, DriveDirection.RIGHT);
                break;
            }
        }
    }
}