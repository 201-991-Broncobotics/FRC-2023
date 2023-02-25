package frc.robot.commands.SetArmPosition;

import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition_Subcommand extends SequentialCommandGroup {
    
    public SetArmPosition_Subcommand(DoubleArm doubleArm, double[] position) {
        double[] target_angles = DoubleArm.getAnglesFromTarget(position[0], position[1]);
        addCommands(
            new SetDistalPositionOne(doubleArm, target_angles[1]), 
            new SetProximalPosition(doubleArm, target_angles[0]), 
            new SetDistalPosition(doubleArm, target_angles[1])
        );
    }
}
