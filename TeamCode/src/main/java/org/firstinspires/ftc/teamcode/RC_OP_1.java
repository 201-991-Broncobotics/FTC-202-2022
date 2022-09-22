package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Systems.*;

@TeleOp(name = "RC Op Mode 1", group = "Iterative Opmode")
public class RC_OP_1 extends LinearOpMode {

  RobotHardware robot = new RobotHardware();

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() throws InterruptedException {
    robot.init(hardwareMap, telemetry);

    waitForStart();
    DriverController driver = new DriverController(robot);
    while (opModeIsActive()) {
      driver.drive(gamepad1);
      driver.inputs(gamepad1);
    }
  }
}
