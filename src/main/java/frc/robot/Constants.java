package frc.robot;

import java.lang.Math;

public final class Constants {

    // general

    public static final boolean show_data = false; // should only be true if use_pigeon2
    public static final boolean use_ADIS16448 = false;
    public static final boolean use_pigeon2 = false;
        // only one of these should be true

    // drivetrain

    public static final double ticks_per_radian = 2048.0 * 12.8 / (2.0 * Math.PI); // 2048 ticks per revolution with a 12.8 L1 gear ratio

    public static final int rf_driving = 3, rf_direction = 2, lf_driving = 4, lf_direction = 5, 
                            rb_driving = 0, rb_direction = 1, lb_driving = 6, lb_direction = 7; // port number
    
    public static final boolean absolute_directing = false; // can only be true if use_ADIS16448 or use_pigeon2 is true

    public static final int imu_port = 0; // only if using pigeon

    // double arm

    public static final double first_pivot_height = 44.875, first_arm_length = 32, second_arm_length = 20; // inches

    public static final int f_pivot = 0, s_pivot = 0; // port number

    public static final double first_arm_initial_angle = -30; // 0 means straight horizontal, positive means counterclockwise
    public static final double second_arm_initial_angle = 30; // 0 means straight in line with first arm, positive means counterclockwise
                                        // both in degrees

    public static final double first_arm_weight = 10; // pounds
    public static final double second_arm_weight = 16; // including the claw

    public static final double first_arm_center = 0.5; // ratio - how far from pivot is the center of mass
    public static final double second_arm_center = 0.6; // should include the claw in center of mass but not in length

    public static final double target_angular_speed = 30; // degrees per second

    // static functions

    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) { angle -= 2.0 * Math.PI; }
        while (angle < 0 - Math.PI) { angle += 2.0 * Math.PI; }

        if (Math.abs(angle) > Math.PI * 0.999999) angle = Math.PI * 0.999999; // if we're at -pi, reset to positive pi

        return angle;
    }

    public static double vectorToAngle(double[] vector) {
        if (vector[0] == 0) vector[0] = 0.001;
        if (vector[1] == 0) vector[1] = 0.001;
        
        return Math.atan(vector[0] / vector[1]) + Math.PI * (vector[1] > 0 ? 0 : 1) * (vector[0] > 0 ? 1 : -1);
    }

    public static double[] angleToVector(double angle) {
        return new double[] {Math.sin(angle), Math.cos(angle)};
    }

    public static void pause(double seconds) {
        double finalTime = System.nanoTime() + seconds * 1000000000L;
        while (System.nanoTime() < finalTime) {
            // idle
        }
    }

    public static void checkIfValuesWork() {
        if (use_pigeon2 && use_ADIS16448) {
            throw new IllegalArgumentException("You can only use one IMU");
        }
        if ((!(use_pigeon2 || use_ADIS16448)) && (show_data || absolute_directing)) {
            throw new IllegalArgumentException("If you're not using an IMU, you can't show your position data and you also can't do absolute direction");
        }
    }
}