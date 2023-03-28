package frc.robot.commands.autoDriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.commands.setArmPosition.SetDistalPosition;
import frc.robot.commands.setClawState.Outtake_Subcommand;
import frc.robot.commands.utilCommands.DriveToPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class DriveToNearestCone extends ConditionalCommand {
    // Y Values:
    // Short Left: 4.98
    // Short Right: 3.87
    // Center Left: 3.3
    // Center Right: 2.18
    // Long Left: 1.63
    // Long Right: 0.5

    // Intial X: 2.1
    // Final X: 1.85

    static double first_x = 2.1, second_x = 1.85;
    // Only do it if we're already in the zone. That is, (x, y) < (2.85, 5.5). Note this has to be adjusted for red
    public DriveToNearestCone(Swerve swerve, DoubleArm doubleArm, Claw claw) {
        super(
            new SequentialCommandGroup(
                condcom(swerve, first_x), 
                new SetArmPosition(doubleArm, cubeTopPositionAngles), 
                condcom(swerve, second_x), 
                new SetDistalPosition(doubleArm, 30), 
                new Outtake_Subcommand(claw), 
                new ParallelCommandGroup(
                    new SetArmPosition(doubleArm, idlePositionAngles), 
                    condcom(swerve, first_x)
                )
            ), 
            new InstantCommand(), 
            () -> {
                Pose2d adjustedPose = Limelight.adjustPoseForSide(swerve.getPose());
                return adjustedPose.getX() < 2.85 && adjustedPose.getY() < 5.5;
            }
        );
    }

    public static Command condcom(Swerve swerve, double x) {
        return 
        new ConditionalCommand(
            new ConditionalCommand(
                new ConditionalCommand(
                    new DriveToPosition(swerve, new Pose2d(x, 4.98, Rotation2d.fromDegrees(180))), 
                    new DriveToPosition(swerve, new Pose2d(x, 3.87, Rotation2d.fromDegrees(180))), 
                    () -> Limelight.adjustPoseForSide(swerve.getPose()).getY() > 4.425
                ), 
                new ConditionalCommand(
                    new DriveToPosition(swerve, new Pose2d(x, 3.3, Rotation2d.fromDegrees(180))), 
                    new DriveToPosition(swerve, new Pose2d(x, 2.18, Rotation2d.fromDegrees(180))), 
                    () -> Limelight.adjustPoseForSide(swerve.getPose()).getY() > 2.74
                ), 
                () -> Limelight.adjustPoseForSide(swerve.getPose()).getY() > 3.585
            ), 
            new ConditionalCommand(
                new DriveToPosition(swerve, new Pose2d(x, 1.63, Rotation2d.fromDegrees(180))), 
                new DriveToPosition(swerve, new Pose2d(x, 0.5, Rotation2d.fromDegrees(180))), 
                () -> Limelight.adjustPoseForSide(swerve.getPose()).getY() > 1.065
            ), 
            () -> Limelight.adjustPoseForSide(swerve.getPose()).getY() > 1.905
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
