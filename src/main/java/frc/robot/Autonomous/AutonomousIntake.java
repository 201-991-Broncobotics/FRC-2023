package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.commands.utilCommands.Drive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class AutonomousIntake extends SequentialCommandGroup {
    public AutonomousIntake(Swerve swerve, DoubleArm doubleArm, Claw claw) {
        addRequirements(swerve, doubleArm, claw);
        // Plan: we drop arm, then drive forward (it will have to be a parallel command group)
        addCommands(
            new SetArmPosition(doubleArm, intakeLowerAngles),
            new ParallelCommandGroup(
                new Drive(swerve, 0.4, 0.2), 
                new Intake(claw)
            )
        );
    }
}