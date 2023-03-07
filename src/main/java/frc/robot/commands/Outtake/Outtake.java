package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPositionV2;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.TuningConstants.*;

public class Outtake extends SequentialCommandGroup {
    
    public Outtake(Claw claw, DoubleArm doubleArm) {
        addRequirements(claw, doubleArm);
        addCommands(
            new Outtake_Subcommand(claw), 
            new SetArmPositionV2(doubleArm, idlePositionAngles)
        );
    }
}
