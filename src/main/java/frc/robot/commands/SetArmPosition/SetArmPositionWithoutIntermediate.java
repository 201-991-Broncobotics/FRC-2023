package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DoubleArm;

public class SetArmPositionWithoutIntermediate extends SequentialCommandGroup {

    public SetArmPositionWithoutIntermediate(DoubleArm doubleArm, double[] position) {
        double[] target_angles = DoubleArm.getAnglesFromTarget(position);
        addCommands(
            new SetMaxDistalPosition(doubleArm, target_angles), 
            new SetProximalPosition(doubleArm, target_angles[0]), 
            new SetDistalPosition(doubleArm, target_angles[1])
        );
    }
    
}