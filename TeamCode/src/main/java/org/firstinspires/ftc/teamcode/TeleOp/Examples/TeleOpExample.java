package org.firstinspires.ftc.teamcode.TeleOp.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;
import static org.firstinspires.ftc.teamcode.Robots.*;

class TeleOpExampleLogic extends TeleOpLogicBase { //You have to change the class name here

    public double starting_time;

    public void execute_non_driver_controlled() {
        //this will have the telemetry, LEDs, etc.

        //Telemetry

        if (useRoadRunner) {
            telemetry.addData("target x", target_x);
            telemetry.addData("target y", target_y);

            telemetry.addData("current x", current_x);
            telemetry.addData("current y", current_y);
        }

        if ((useRoadRunner) || (usePID)) {
            telemetry.addData("target angle", target_angle);
            telemetry.addData("current angle", current_angle);

            telemetry.addData("angle to field", angle());
        }

        telemetry.addData("cycles per second", 0.1 * ((int) (10 / delta_time)));
        telemetry.addData("elapsed time", 0.1 * ((int) (10.0 * (starting_time - current_time))));


        //robot.telemetry.addData("Arm Position: ", robot.dc_motor_list[dc_motor_names.indexOf("arm")].getCurrentPosition());

        /* EXAMPLE - set LED color if distance sensor detects something
        if (robot.getDistInch("dSensor") < 4) robot.set_led_color("led", "Blue");
        else robot.set_led_color("led", "Green");
        EXAMPLE - set LED color if a is pressed
        if (buttons[keys.indexOf("operator a")]) robot.setLed("led", "Blue"); //note you have to subtract 20 if you want to access axis value
        else robot.setLed("led", "Green");
         */

        telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init(HardwareMap hm, Telemetry tm) {
        init_base();
        starting_time = System.nanoTime() / 1000000000.0;
        initialize_logic(hm, tm);
        setZeroAngle(-90); //Relative to Driver, Positive = Clockwise
        //ex. if robot facing left, then starting angle = -90
        //target_positions[dc_motor_names.size() + servo_names.indexOf("right")] = 1.0;
        set_keybinds();
        set_button_types();
        setZeroAngle(-90); //Relative to Driver, Positive = Clockwise
        //ex. if robot facing left, then starting angle = -90
        //target_positions[dc_motor_names.size() + servo_names.indexOf("right")] = 1.0;
        button_types[keys.indexOf("operator a")] = 3; //1 is default, 2 is toggle, 3 is button
    }

    public void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 90, localizer);
        //Direction Robot is facing; if facing left, then it is either -90° or 90°
    }

    public void set_keybinds() {

        //arm

        //new_keybind("arm", "operator right_stick_y", "default", 0.26, 0.13);
        //new_keybind("arm", "driver left_trigger", "cycle", 1, armPositions);

        //intake

        //new_keybind("intake", "driver a", "toggle", "normal", 0.3);
        //new_keybind("intake", "driver y", "toggle", "normal", -0.3);

        //duck

        //new_keybind("duckWheel", "driver b", "toggle", "gradient", 0.7);
        //new_keybind("duckWheel", "driver x", "toggle", "gradient", -0.7);

        //right

        //new_keybind("right", "driver dpad_up", "cycle", 1, servoPositions);

        //driver

        //new_keybind("goto", "driver right_bumper", 50, 80, "none");

    }


}

@TeleOp(name="TeleOp Example", group="Iterative Opmode") //CHANGE THIS
@Disabled
public class TeleOpExample extends LinearOpMode {
    TeleOpExampleLogic logic = new TeleOpExampleLogic();
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        waitForStart();
        if (useRoadRunner) {
            logic.initRoadRunner(new StandardTrackingWheelLocalizer(hardwareMap, logic));
        }
        while (opModeIsActive()) {
            logic.tick(gamepad1, gamepad2); //driver is gamepad1, operator is gamepad2
            logic.execute_non_driver_controlled();
        }
    }
}