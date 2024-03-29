package org.firstinspires.ftc.teamcode.Logic.AutonomousLogic;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Logic.RobotHardware;

public class ThreadedServo extends Thread {

    Servo servo;
    double target_position;

    public ThreadedServo(HardwareMap map, String servo_name) {
        servo = map.get(Servo.class, servo_name);
    }

    public void set_position(double p) {
        target_position = p;
    }

    public boolean should_be_running = true;

    public void run() {
        while (should_be_running) {
            servo.setPosition(target_position);
        }
    }

}