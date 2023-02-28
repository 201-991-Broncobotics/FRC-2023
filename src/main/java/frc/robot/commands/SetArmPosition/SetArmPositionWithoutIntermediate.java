package frc.robot.commands.setArmPosition;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DoubleArm;

public class SetArmPositionWithoutIntermediate extends SequentialCommandGroup {

    public SetArmPositionWithoutIntermediate(DoubleArm doubleArm, double[] position) {
        addRequirements(doubleArm);
        double[] target_angles = DoubleArm.getAnglesFromTarget(position);
        addCommands(
            new SetMaxDistalPosition(doubleArm, target_angles), 
            new SetProximalPosition(doubleArm, target_angles[0]), 
            new SetDistalPosition(doubleArm, target_angles[1])
        );
    }

    public SetArmPositionWithoutIntermediate(DoubleArm doubleArm, double[] position, BooleanSupplier stopSup) {
        addRequirements(doubleArm);
        double[] target_angles = DoubleArm.getAnglesFromTarget(position);
        addCommands(
            new SetMaxDistalPosition(doubleArm, target_angles, stopSup), 
            new SetProximalPosition(doubleArm, target_angles[0], stopSup), 
            new SetDistalPosition(doubleArm, target_angles[1], stopSup)
        );
    }
}