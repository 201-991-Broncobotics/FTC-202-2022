package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.AprilTag;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.OpenCV;

@Autonomous(name="april tag vision test")
public class VisionTest extends LinearOpMode {
    AprilTag aprilTag = new AprilTag();
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag.init(hardwareMap, telemetry);

        aprilTag.startScanning();

        waitForStart();
        aprilTag.stopScanning();
    }
}
