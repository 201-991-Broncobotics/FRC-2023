package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetArmPosition.SetArmPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import java.util.function.BooleanSupplier;

import static frc.robot.Constants.TuningConstants.*;

public class Intake extends SequentialCommandGroup {
    public Intake(Claw claw, DoubleArm doubleArm, BooleanSupplier stopSup) {
        addRequirements(claw); // means that other functions are not allowed to access it
        addCommands(
            new Intake_Subcommand(claw, stopSup), 
            new SetArmPosition(doubleArm, idlePosition)
        );
    }
}
