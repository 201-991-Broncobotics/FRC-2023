package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

import static frc.robot.autos.autoTrajectories.*;

public class Autonomous extends SequentialCommandGroup {
    public Autonomous(Claw claw, DoubleArm doubleArm, Swerve swerve) {
        addRequirements(doubleArm, claw, swerve);
        addCommands(
            new emulatorAuto(swerve, testTrajectory)
        );
    }
}
