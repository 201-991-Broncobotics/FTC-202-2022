package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Systems.*;

@TeleOp(name = "RC Op Mode 1", group = "Iterative Opmode")
public class RC_OP_1 extends LinearOpMode {

  RobotHardware robot = new RobotHardware();

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode()  {
    robot.init(hardwareMap, telemetry);

    waitForStart();
    DriverController driver = new DriverController(robot);
    ObjectiveController objective = new ObjectiveController(robot);
    while (opModeIsActive()) {
      driver.drive(gamepad1);
      driver.inputs(gamepad1);
      objective.scuffedArm(gamepad2);
    }
  }
}
