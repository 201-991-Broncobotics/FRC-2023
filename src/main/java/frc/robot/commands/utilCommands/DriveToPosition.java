package frc.robot.commands.utilCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DriveToPosition extends CommandBase {
    // if red, y becomes 8.02 - y (because 8.02 is the field width in meters)

    // main poi: the single substation is around (14.1, 7.08) with rotation of 90 (we then drive right until we are tight against it)
    // y should be 0.96

    private Swerve swerve;
    private final Pose2d targetPose;
    private Translation2d targetPosition;
    private final double p = 2.8, e = 1.05, tolerance = 0.035, ang_tolerance = 1.2, maxSpeedHere = 0.8, calitime = 0.5;
    private double end_time = 0;

    public DriveToPosition(Swerve swerve, Pose2d targetPose) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        // Limelight.setSide("blue"); todo figure this out
        targetPosition = new Translation2d(targetPose.getX(), Limelight.getSide().equals("blue") ? targetPose.getY() : 8.02 - targetPose.getY());
        swerve.setTargetHeading(Limelight.getSide().equals("blue") ? targetPose.getRotation().getDegrees() : -targetPose.getRotation().getDegrees());
        SmartDashboard.putString("Target Pose", "(" + Math.round(targetPosition.getX() * 100) / 100.0 + ", " + Math.round(targetPosition.getY() * 100) / 100.0 + ")");
        end_time = Timer.getFPGATimestamp() + calitime;
    }

    @Override
    public void execute() {
        Translation2d error = targetPosition.minus(swerve.poseEstimator.getEstimatedPosition().getTranslation());
        error = error.times(p);
        if (error.getNorm() > 1) error = error.times(1 / error.getNorm());
        else error = error.times(Math.pow(error.getNorm(), e - 1));
        swerve.drive(
            error.times(Constants.BaseFalconSwerve.maxSpeed).times(maxSpeedHere), // PID I BELIEVE
            0, 
            true, 
            true, 
            false
        );
        if (!((targetPosition.minus(swerve.poseEstimator.getEstimatedPosition().getTranslation()).getNorm() < tolerance) && (swerve.getError() < ang_tolerance))) {
            end_time = Timer.getFPGATimestamp() + calitime;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > end_time;
    }
}
