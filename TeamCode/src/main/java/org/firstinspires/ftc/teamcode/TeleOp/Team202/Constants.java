package org.firstinspires.ftc.teamcode.TeleOp.Team202;

public final class Constants {
    public static final String test = "test";

    public static final double  first_arm_zero = 410, second_arm_zero = -817;

    public static final double ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI;

    public static final double lift_speed = 1.3, in_speed = 1.3, stick_lift_speed = 2, stick_in_speed = 2;

    public static final double claw_open = 0.5, claw_closed = 1;

    public static final double wrist_m = 0.00334, wrist_b = 0.33577+.24;

    public static final double wrist_angle_speed = 1.0;

    public static double ensureBetween(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }

    public static int ensureBetween(int val, int min, int max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}
