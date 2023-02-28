package frc.robot.commands.setArmPosition;

import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.TuningConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition extends SequentialCommandGroup {
    
    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        addRequirements(doubleArm);
        addCommands(
            new SetArmPositionWithoutIntermediate(doubleArm, intermediatePosition), 
            new SetArmPositionWithoutIntermediate(doubleArm, position)
        );
    }

    public SetArmPosition(DoubleArm doubleArm, double[] position, BooleanSupplier stopSup) {
        addRequirements(doubleArm);
        addCommands(
            new SetArmPositionWithoutIntermediate(doubleArm, intermediatePosition, stopSup), 
            new SetArmPositionWithoutIntermediate(doubleArm, position, stopSup)
        );
    }
}