package org.firstinspires.ftc.teamcode.TeleOp.Team202;

public final class Constants {
    public static final double  first_arm_zero = 410, second_arm_zero = -817;

    public static final double ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI;

    public static final double lift_speed = 1.3, in_speed = 1.3, stick_lift_speed = 2, stick_in_speed = 2;

    public static final double claw_open = 0.3, claw_closed = 1;

    public static final double wrist_m = 0.00334, wrist_b = 0.33577 + .24 + 0.04;

    public static final double wrist_angle_speed = 1.0;

    public static final int tag1id = 0, tag2id = 2, tag3id = 6;

    public static final double DRIVETRAIN_TICKS_PER_REV = ((((1+(46.0/17.0))) * (1+(46.0/11.0))) * 28.0);

    // 25.4 mm/in * ticks per rev / (pi * wheel diameter)
    public static final double DRIVETRAIN_TICKS_PER_INCH = (25.4 * DRIVETRAIN_TICKS_PER_REV / (96.0 * Math.PI));

    public static final double EXTRA_STRAFE_TICKS = 9.0 / 8.0;

    public static final double robot_width = 14.423228;
    public static final double robot_length = 14.373228;

    public static final double IN_PER_MM = 1.0 / 25.4;

    /** how far forward the camera is from the center of the robot */
    public static final double CAMERA_FROM_CENTER_Y = 25 * IN_PER_MM - robot_length / 2;
    /** how far right of the center the camera lens is */
    public static final double CAMERA_FROM_CENTER_X = robot_width / 2 - (27.5 * IN_PER_MM + 1.889764);

    public static final double ARM_FROM_CENTER = - 8.505395 + robot_length / 2;

    public static final double ARM_LENGTH = 11.338583, /** complete guess :D */ CLAW_OFFSET = 8.75;

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
