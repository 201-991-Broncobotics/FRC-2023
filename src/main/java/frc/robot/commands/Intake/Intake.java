package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPositionV2;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.TuningConstants.*;

public class Intake extends SequentialCommandGroup {
    
    public Intake(Claw claw, DoubleArm doubleArm, DoubleSupplier motorOneSup, DoubleSupplier motorTwoSup) {
        addRequirements(claw, doubleArm);
        addCommands(
            new Intake_Subcommand(claw, doubleArm, motorOneSup, motorTwoSup), 
            new SetArmPositionV2(doubleArm, idlePositionAngles)
        );
    }
}
