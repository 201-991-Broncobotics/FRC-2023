package frc.robot.commands.setArmPosition;

import java.util.function.BooleanSupplier;

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

    public SetArmPositionWithoutIntermediate(DoubleArm doubleArm, double[] target_angles, BooleanSupplier stopSup) {
        addRequirements(doubleArm);
        addCommands(
            new SetMaxDistalPosition(doubleArm, target_angles[0], stopSup), 
            new SetProximalPosition(doubleArm, target_angles[0], stopSup), 
            new SetDistalPosition(doubleArm, target_angles[1], stopSup)
        );
    }
}