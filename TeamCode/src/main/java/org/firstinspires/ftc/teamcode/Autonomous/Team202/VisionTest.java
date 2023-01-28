package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.OpenCV;

@Autonomous(name="vision test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpenCV camera = new OpenCV(hardwareMap, telemetry, "Webcam 1");

        camera.start();

        waitForStart();

//        int runs = 0;
//        while (opModeIsActive()) {
//            runs ++;
//        }

//        camera.stopStreaming();
    }
}
