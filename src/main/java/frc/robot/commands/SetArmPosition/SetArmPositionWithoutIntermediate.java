package frc.robot.commands.setArmPosition;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
            new InstantCommand(() -> SmartDashboard.putString("Status", "Set Max Distal")),
            new SetMaxDistalPosition(doubleArm, target_angles[0], stopSup), 
            new InstantCommand(() -> SmartDashboard.putString("Status", "Set Proximal")),
            new SetProximalPosition(doubleArm, target_angles[0], stopSup), 
            new InstantCommand(() -> SmartDashboard.putString("Status", "Set Final Distal")),
            new SetDistalPosition(doubleArm, target_angles[1], stopSup),
            new InstantCommand(() -> SmartDashboard.putString("Status", "Finished Setting Arm Target"))
        );
    }
}