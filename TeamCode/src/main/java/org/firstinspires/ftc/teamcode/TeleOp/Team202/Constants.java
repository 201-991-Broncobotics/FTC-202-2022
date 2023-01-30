package org.firstinspires.ftc.teamcode.TeleOp.Team202;

public final class Constants {
    public static final double  first_arm_zero = 410, second_arm_zero = -817;

    public static final double ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI;

    public static final double lift_speed = 1.3, in_speed = 1.3, stick_lift_speed = 2, stick_in_speed = 2;

    public static final double claw_open = 0.3, claw_closed = 0.9;

    public static final double wrist_m = 0.00334, wrist_b = 0.33577 + .24 + 0.04;

    public static final double wrist_angle_speed = 1.0;

    public static final int tag1id = 0, tag2id = 5, tag3id = 9;

    public static final double DRIVETRAIN_TICKS_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/11.0))) * 28.0);

    // 25.4 mm/in * ticks per rev / (2 * pi * wheel diameter)
    public static final double DRIVETRAIN_TICKS_PER_INCH = 25.4 * DRIVETRAIN_TICKS_PER_REV / (2.0 * Math.PI * 96.0);

    public static final double robot_width = 10.393701;
    public static final double robot_length = 14.373228;

    public static final double CAMERA_FROM_CENTER = 12.688189 - robot_length / 2;

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
