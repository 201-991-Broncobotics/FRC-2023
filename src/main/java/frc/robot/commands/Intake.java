package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

public class Intake extends SequentialCommandGroup {
    public Intake(Claw claw, DoubleArm doubleArm, BooleanSupplier intakeSup) {
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it
        addCommands(
            new Intake_Subcommand_One(claw, doubleArm), 
            new Intake_Subcommand_Two(claw, doubleArm, intakeSup)
        );
    }
}
