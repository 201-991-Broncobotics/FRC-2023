package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setClawState.Intake;
import frc.robot.commands.utilCommands.Drive;
import frc.robot.commands.utilCommands.Wait;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class AutonomousIntake extends SequentialCommandGroup {
    public AutonomousIntake(Swerve swerve, Claw claw) {
        addRequirements(swerve, claw);
        // Plan: we drop arm, then drive forward (it will have to be a parallel command group)
        addCommands(
            new ParallelDeadlineGroup(
                new Wait(2), 
                new ParallelDeadlineGroup(
                    new Intake(claw), 
                    new Drive(swerve, 0.4, 0.2)
                )
            )
        );
    }
}