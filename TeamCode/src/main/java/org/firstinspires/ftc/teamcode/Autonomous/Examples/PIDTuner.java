package org.firstinspires.ftc.teamcode.Autonomous.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.RobotHardware;

@Autonomous(name = "PD Tuner")
@Disabled
public class PIDTuner extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    StandardTrackingWheelLocalizer position_tracker;

    double current_time = System.currentTimeMillis();
    double previous_time = System.currentTimeMillis();

    double current_error = 0.0;
    double previous_error = 0.0;

    double error = 0;
    double target = Math.PI / 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize_hardware(hardwareMap, telemetry);
        if (robot.useRoadRunner) {
            position_tracker = new StandardTrackingWheelLocalizer(hardwareMap, robot);
        } else if (!robot.usePID) {
            throw new IllegalArgumentException("Can't tune PID if not using PID or RoadRunner");
        }

        waitForStart();

        target = Math.PI / 2.0; //Turn 90 degrees right

        while (Math.abs(error) > 0.02) { //ONE degree
            turn(Math.max(-1, Math.min(1,
                    getCorrection()
            )));
            error = target;
            if (robot.useRoadRunner) {
                position_tracker.update();
                error += position_tracker.getPoseEstimate().getHeading();
            } else {
                error += robot.getAngle() * Math.PI / 180.0;
            }
            error = modulus(error + Math.PI, 2.0 * Math.PI) - Math.PI;
        }

        stop();
    }

    public void turn(double speed) {
        for (int i = 0; i < 4; i++) {
            robot.wheel_list[i].setPower(speed * ((i > 1) ? -1 : 1));
        }
    }

    public double getCorrection() {
        current_time = System.currentTimeMillis();
        current_error = error;

        double p = current_error;
        double d = (current_error - previous_error) / (current_time - previous_time);

        previous_error = current_error;
        previous_time = current_time;

        return robot.p_weight * p + robot.d_weight * d;
    }

    public double modulus(double value, double base) {
        while (value >= base) {
            value -= base;
        }
        while (value < base) {
            value += base;
        }
        return value;
    }

}