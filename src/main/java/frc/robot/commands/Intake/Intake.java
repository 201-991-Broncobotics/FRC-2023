package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetArmPosition.SetArmPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.TuningConstants.*;

public class Intake extends SequentialCommandGroup {
    public Intake(Claw claw, DoubleArm doubleArm) {
        addRequirements(claw); // means that other functions are not allowed to access it
        addCommands( // Let's see if this works lmao+
            new SetArmPosition(doubleArm, intakePosition), 
            new Intake_Subcommand(claw, doubleArm)
        );
    }
}
