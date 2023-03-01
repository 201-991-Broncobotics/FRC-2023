package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPositionWithoutIntermediate;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.TuningConstants.*;

public class Intake extends SequentialCommandGroup {
    
    public Intake(Claw claw, DoubleArm doubleArm) {
        addRequirements(claw, doubleArm);
        addCommands(
            new Intake_Subcommand(claw), 
            new SetArmPositionWithoutIntermediate(doubleArm, idlePositionAngles)
        );
    }
}
