package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.commands.utilCommands.Drive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class Outtake extends SequentialCommandGroup {
    
    public Outtake(Swerve swerve, Claw claw, DoubleArm doubleArm) {
        addRequirements(claw, doubleArm);
        addCommands(
            new Outtake_Subcommand(claw), 
            new Drive(swerve, 0.5, -0.6), 
            new SetArmPosition(doubleArm, idlePositionAngles)
        );
    }
}
