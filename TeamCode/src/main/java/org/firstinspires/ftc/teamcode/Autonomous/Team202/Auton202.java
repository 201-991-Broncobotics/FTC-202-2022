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

        aprilTag.startScanning();

        logic.closeClaw();

        waitForStart();

        // left of square to center of square
        logic.driveInches(12 - robot_width / 2, DriveDirection.RIGHT);
        sleep(4500);

        // stop right in front of the cone with sleeve
        logic.driveInches(36 - robot_length / 2, DriveDirection.FORWARD);
        sleep(2000);

        aprilTag.stopScanning();
        int id = aprilTag.getMostDetected();

        logic.driveInches(24, DriveDirection.FORWARD);
        sleep(1000);

        logic.initArm();

        sleep(3000);

        final double armX = Math.sqrt(4 - 1.93 * 1.93 - .01);

        logic.setArmPos(armX,1.93);
        sleep(2000);

        // the cone will be ARM_FROM_CENTER + Math.sqrt(4 - 1.93 * 1.93 - .01) * ARM_LENGTH + CLAW_OFFSET
        // from the center of the robot, so we need to adjust for that.

        // ~14
        final double CONE_FROM_CENTER = ARM_FROM_CENTER + armX * ARM_LENGTH + CLAW_OFFSET;

        logic.driveInches(CONE_FROM_CENTER - 12, DriveDirection.BACKWARD);
        sleep(1000);

        logic.driveInches(12, DriveDirection.RIGHT);
        sleep(1000);

        // theoretically, we should be right above the pole rn...

        // to help drop the cone onto the pole
        logic.setArmPos(armX, 1.85);


        logic.openClaw();

        logic.setArmPos(0.3, -0.6); // op x in teleop

        sleep(2000);

        logic.driveInches(CONE_FROM_CENTER - 12, DriveDirection.FORWARD);
        sleep(500);

        switch (id) {
            case tag1id: {
                logic.driveInches(36, DriveDirection.LEFT);
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