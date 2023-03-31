package frc.robot.commands.autoDriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.commands.setClawState.Intake;
import frc.robot.commands.utilCommands.DriveToPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class DriveToNearestIntakeStation extends ConditionalCommand {
    public DriveToNearestIntakeStation(Swerve swerve, DoubleArm doubleArm, Claw claw) {
        super(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new DriveToPosition(swerve, new Pose2d(15, 7.34, new Rotation2d())), 
                        new SetArmPosition(doubleArm, cubeTopPositionAngles), 
                        new ParallelDeadlineGroup(
                            new Intake(claw), 
                            new DriveToPosition(swerve, new Pose2d(15.5, 7.34, new Rotation2d()))
                        )
                    ), 
                    new SequentialCommandGroup(
                        new DriveToPosition(swerve, new Pose2d(15, 6.02, new Rotation2d())), 
                        new SetArmPosition(doubleArm, cubeTopPositionAngles), 
                        new ParallelDeadlineGroup(
                            new Intake(claw), 
                            new DriveToPosition(swerve, new Pose2d(15.5, 6.02, new Rotation2d()))
                        )
                    ), 
                    () -> Limelight.adjustPoseForSide(swerve.getPose()).getY() > 6.68
                ), 
                new ParallelCommandGroup(
                    new SetArmPosition(doubleArm, intakeUpperAngles), 
                    new DriveToPosition(swerve, new Pose2d(14.7, 6.68, new Rotation2d()))
                ), 
                new InstantCommand(() -> swerve.setTargetHeading(180))
            ), 
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new DriveToPosition(swerve, new Pose2d(14.1, 7.08, Rotation2d.fromDegrees(90))), 
                    new ParallelDeadlineGroup(
                        new Intake(claw), 
                        new DriveToPosition(swerve, new Pose2d(14.1, 7.5, Rotation2d.fromDegrees(90)))
                    ), 
                    new DriveToPosition(swerve, new Pose2d(14.1, 6.68, Rotation2d.fromDegrees(180)))
                ), 
                new InstantCommand(), 
                () -> {
                    Pose2d adjustedPose = Limelight.adjustPoseForSide(swerve.getPose());
                    return (adjustedPose.getX() > 10.3 && adjustedPose.getY() > 5.8);
                }
            ), 
            () -> {
                Pose2d adjustedPose = Limelight.adjustPoseForSide(swerve.getPose());
                if (adjustedPose.getX() < 10.3 || adjustedPose.getY() < 5.8) return false; // out of bounds
                if (onlySingle) return false;
                if (Math.abs(adjustedPose.getRotation().getDegrees()) < 15) return false; // we're not facing in that direction so probably single
                if (adjustedPose.getX() < 14.5) return false; // we're not close enough so probably single
                return true;
            }
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}