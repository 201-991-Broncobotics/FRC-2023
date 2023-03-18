package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { // NOT a subsystem

    private static DoubleSupplier tv = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(-1); // 0 --> nothing, 1 --> something
    // private static DoubleSupplier tx = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(-100);
    // private static DoubleSupplier ty = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(-100);
    // private static DoubleSupplier ta = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(-1);

    private static DoubleSupplier tid = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-2);
    
    private static DoubleSupplier[] position = new DoubleSupplier[] {
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[4], 
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[0], 
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[2]
    }; // Where the camera is, relative to april tag
    
    private static String side = "blue";
    public static void setSide(String side) {
        if (side.toLowerCase().equals("red")) {
            side = "red";
        } else if (side.toLowerCase().equals("blue")) {
            side = "blue";
        }
    }

    /** x, y (meters), yaw (degrees), latency (seconds) */
    private static DoubleSupplier[] botpose = new DoubleSupplier[] {
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpi" + side).getDoubleArray(new double[7])[0], // x
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpi" + side).getDoubleArray(new double[7])[1], // y
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpi" + side).getDoubleArray(new double[7])[5], // yaw
        () -> (
                (
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0) + 
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0)
                ) / 1000.0) // latency
    }; // Where the robot is, relative to april tag

    /*
     * First value should match what we receive from LeYaw
     * Second value is how far we are horizontally
     * Third value is how far we are out from it
     * Ex. if we were 6m away from the april tag, 2m to the right, and facing at a 10 degree angle to the right of it the measurements would read:
     * [10, -2, 6] I THINK
     * We probably will never be 6m away though lmao
     * Signs might be off, and I'm not sure if it uses radians or degrees
     * 
     * April tag width: 6 in = 152.4 mm
     */

    public static void init() {
        setPipeline(0);
        turnOffLED();
    }

    public static double[] getData() {
        if (tv.getAsDouble() == 0) return new double[] {-12, -12, -12, -12};
        return new double[] {
            tid.getAsDouble(), 
            position[0].getAsDouble(), 
            position[1].getAsDouble(), 
            position[2].getAsDouble()
        };
    }

    public static Pose2d getRobotPosition() {
        double x = botpose[0].getAsDouble();
        double y = botpose[1].getAsDouble();
        double angle = botpose[2].getAsDouble();

        SmartDashboard.putString("Limelight Ping", "" + Math.round(botpose[3].getAsDouble() * 1000) + " ms");

        return new Pose2d(x, y, Rotation2d.fromDegrees(angle)); 
    }

    public static double getLatency() {
        return botpose[3].getAsDouble(); 
    }

    public static void setPipeline(int number) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(number); // which pipeline we use
    }

    public static void turnOffLED() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
}