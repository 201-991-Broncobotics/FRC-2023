package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setClawState.Intake;
import frc.robot.commands.utilCommands.Drive;
import frc.robot.commands.utilCommands.Wait;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class AutonomousIntake extends ParallelDeadlineGroup {
    public AutonomousIntake(Swerve swerve, Claw claw) {
        super(
            new Wait(5), 
            new Intake(claw)
        );
    }
}