package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Arm Test")
public class ArmTestAuto extends LinearOpMode {
    AutonLogic202 logic = new AutonLogic202();
    @Override
    public void runOpMode() {
        telemetry.addLine("here");
        telemetry.update();

        logic.init(hardwareMap, telemetry);

        telemetry.addLine("here1");
        telemetry.update();

        telemetry.addLine("here1");
        telemetry.update();

        waitForStart();

        telemetry.addLine("here3");
        telemetry.update();

        logic.initArm();

        telemetry.addLine("here4");
        telemetry.update();

        logic.setArmPos(1.25, 1.25);

        telemetry.addLine("here5");
        telemetry.update();

        sleep(10000);

        telemetry.addLine("here6");
        telemetry.update();

        logic.arm.quit();

    }

}



