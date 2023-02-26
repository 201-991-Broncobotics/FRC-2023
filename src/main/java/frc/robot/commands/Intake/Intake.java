package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPositionWithoutIntermediate;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.TuningConstants.*;

public class Intake extends SequentialCommandGroup {
    public Intake(Claw claw, DoubleArm doubleArm) {
        addRequirements(claw); // means that other functions are not allowed to access it
        addCommands(
            new Intake_Subcommand(claw), 
            new SetArmPositionWithoutIntermediate(doubleArm, idlePosition)
        );
    }
}
