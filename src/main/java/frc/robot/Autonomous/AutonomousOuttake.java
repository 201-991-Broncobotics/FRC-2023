package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setClawState.Outtake;
import frc.robot.commands.utilCommands.Brake;
import frc.robot.commands.utilCommands.Drive;
import frc.robot.commands.utilCommands.Wait;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

public class AutonomousOuttake extends SequentialCommandGroup {
    public AutonomousOuttake(Swerve swerve, DoubleArm doubleArm, Claw claw) {
        addRequirements(swerve, doubleArm, claw);
        addCommands(
            new Drive(swerve, 0.2, 0.4), 
            new Brake(swerve), 
            new Wait(0.2), 
            new Outtake(claw, swerve)
        );
    }
}
