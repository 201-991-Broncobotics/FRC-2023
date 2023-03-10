package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { // NOT a subsystem

    private static DoubleSupplier tv = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(-1); // 0 --> nothing, 1 --> something
    // private static DoubleSupplier tx = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(-100);
    // private static DoubleSupplier ty = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(-100);
    // private static DoubleSupplier ta = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(-1);
    
    private static DoubleSupplier tl = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(-1);

    private static DoubleSupplier tid = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-2);
    
    private static DoubleSupplier[] position = new DoubleSupplier[] {
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[4], 
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[0], 
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[2]
    }; // Where the camera is, relative to april tag

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

    public static void displayData() {
        double[] vals = getData();
        SmartDashboard.putNumber("April Tag ID", vals[0]);
        SmartDashboard.putNumber("Robot Heading", vals[1]);
        SmartDashboard.putNumber("Robot x", vals[2]);
        SmartDashboard.putNumber("Robot z", vals[3]);
    }

    public static double[] getRobotPosition() {
        double x = 0;
        double y = 0;
        double angle = 0;
        // maybe we augment it with yaw but idk
        double latency = tl.getAsDouble();
        return new double[] {x, y, angle, latency};
    }

    public static void setPipeline(int number) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(number); // which pipeline we use
    }

    public static void turnOffLED() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
}