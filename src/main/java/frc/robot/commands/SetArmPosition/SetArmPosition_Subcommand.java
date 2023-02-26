package frc.robot.commands.setArmPosition;

import frc.robot.subsystems.DoubleArm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition_Subcommand extends SequentialCommandGroup {
    
    public SetArmPosition_Subcommand(DoubleArm doubleArm, double[] position) {
        double[] target_angles = DoubleArm.getAnglesFromTarget(position);
        addCommands(
            new SetMaxDistalPosition(doubleArm, target_angles), 
            new SetProximalPosition(doubleArm, target_angles[0]), 
            new SetDistalPosition(doubleArm, target_angles[1])
        );
    }
}
