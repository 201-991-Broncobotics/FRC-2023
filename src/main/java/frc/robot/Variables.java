package frc.robot;

public class Variables {
    // Unlike constants, these variables are... well... variable
    public static double speed_factor = 1.0;

    public static double angular_offset = 0, 
                         prev_x_average = 0,
                         target_swerve_heading = 0;
    
    public static double[] previous_x = new double[Constants.AprilTagAlignmentConstants.distance_trials];

    public static boolean continueWithAWA = true;

    public static boolean go_to_startposition = false;

    public static String data = "";

    public static double ats = 0;
    public static boolean thor = false;
}
