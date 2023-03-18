package frc.robot.commands.utilCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private final double p = 0.35, tolerance = 0.15, ang_tolerance = 360, maxSpeedHere = 0.8;

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
    }

    @Override
    public void execute() {
        Translation2d error = targetPosition.minus(swerve.poseEstimator.getEstimatedPosition().getTranslation());
        error = error.times(-p);
        if (error.getNorm() > 1) error.times(1 / error.getNorm());
        swerve.drive(
            error.times(Constants.BaseFalconSwerve.maxSpeed).times(maxSpeedHere), // PID I BELIEVE
            0, 
            true, 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }

    @Override
    public boolean isFinished() {
        return (targetPosition.minus(swerve.poseEstimator.getEstimatedPosition().getTranslation()).getNorm() < tolerance) && (swerve.getError() < ang_tolerance);
    }
}
