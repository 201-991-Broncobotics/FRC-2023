package frc.robot.commands.SetArmPosition;

import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.TuningConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition extends SequentialCommandGroup {
    
    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        addCommands(
            new SetArmPosition_Subcommand(doubleArm, intermediatePosition), 
            new SetArmPosition_Subcommand(doubleArm, position)
        );
    }
}