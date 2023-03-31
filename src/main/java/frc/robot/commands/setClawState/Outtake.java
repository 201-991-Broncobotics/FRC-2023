package frc.robot.commands.setClawState;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utilCommands.Drive;
import frc.robot.commands.utilCommands.Wait;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DoubleArm;

public class Outtake extends SequentialCommandGroup {
    
    public Outtake(Claw claw, Swerve swerve) {
        addRequirements(claw, swerve);
        addCommands(
            new Outtake_Subcommand(claw), 
            new Wait(0.2), 
            new ConditionalCommand(
                new Drive(swerve, 0.75, -0.55), 
                new Drive(swerve, 0.5, -0.55), 
                DoubleArm.isConeMode
            ), 
            new InstantCommand(() -> frc.robot.Variables.go_to_startposition = true)
        );
    }
}