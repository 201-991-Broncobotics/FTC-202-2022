package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class CenterWrist extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo wrist = hardwareMap.get(Servo.class, "clawAligner");
        wrist.setPosition(0.5);
    }
}
