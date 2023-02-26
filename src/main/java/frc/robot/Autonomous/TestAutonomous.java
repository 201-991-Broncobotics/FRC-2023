package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

public class TestAutonomous extends SequentialCommandGroup {
    public TestAutonomous(Claw claw, DoubleArm doubleArm, Swerve swerve) {

        addRequirements(doubleArm, claw, swerve);

        addCommands(
            new RunTrajectory(swerve, "test")
        );
    }
}
