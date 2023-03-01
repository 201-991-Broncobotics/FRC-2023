package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DoubleArm;

public class SetArmPositionWithoutIntermediate extends SequentialCommandGroup {

    public SetArmPositionWithoutIntermediate(DoubleArm doubleArm, double[] target_angles) {
        addRequirements(doubleArm);
        addCommands(
            new SetMaxDistalPosition(doubleArm, target_angles[0]), 
            new SetProximalPosition(doubleArm, target_angles[0]), 
            new SetDistalPosition(doubleArm, target_angles[1])
        );
    }
}