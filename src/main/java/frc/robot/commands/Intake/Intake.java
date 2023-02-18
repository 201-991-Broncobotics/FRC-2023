package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetArmPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.TuningConstants.*;

public class Intake extends SequentialCommandGroup {
    public Intake(Claw claw, DoubleArm doubleArm, BooleanSupplier intakeSup) {
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it
        addCommands(
            new SetArmPosition(doubleArm, intakePosition), 
            new Intake_Subcommand(claw, doubleArm, intakeSup)
        );
    }
}
