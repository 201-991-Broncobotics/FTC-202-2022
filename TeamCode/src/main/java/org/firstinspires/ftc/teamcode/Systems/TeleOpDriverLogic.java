package org.firstinspires.ftc.teamcode.Systems;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class TeleOpDriverLogic {

  double LeftBeam = 0.3;
  double RightBeam = 0.6;

  RobotHardware robot;
  DriverController driver;

  public TeleOpDriverLogic(RobotHardware r, DriverController d) {
    robot = r;
    driver = d;
  }
}
