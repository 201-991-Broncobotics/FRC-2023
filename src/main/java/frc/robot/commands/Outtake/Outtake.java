package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utilCommands.Drive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class Outtake extends SequentialCommandGroup {
    
    public Outtake(Claw claw, Swerve swerve) {
        addRequirements(claw, swerve);
        addCommands(
            new Outtake_Subcommand(claw), 
            new Drive(swerve, 0.5, -0.4), 
            new InstantCommand(() -> frc.robot.Variables.go_to_startposition = true)
        );
    }
}