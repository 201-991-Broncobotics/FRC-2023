package frc.robot.commands.autoDriveCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DriveToDoubleSubstation extends ConditionalCommand {
    // Y Values:
    // Left: 7.34
    // Right: 6.02

    // Initial x: 15?
    // Final X: honestly idk
    public DriveToDoubleSubstation(Swerve swerve) {
        super(
            new InstantCommand(), 
            new InstantCommand(), 
            () -> swerve.getPose().getX()
        );
    }
}