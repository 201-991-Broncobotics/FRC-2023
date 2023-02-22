package frc.robot;

public class Variables {
    // Unlike constants, these variables are... well... varible
    public static double speed_factor = 1.0;

    public static double[] previous_angles = new double[Constants.AprilTagAlignmentConstants.angle_trials];
    public static double[] previous_x = new double[Constants.AprilTagAlignmentConstants.distance_trials];

    public static boolean in_autoalignment = false;
}
