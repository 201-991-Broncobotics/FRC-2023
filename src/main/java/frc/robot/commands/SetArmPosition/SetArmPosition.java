package frc.robot.commands.setArmPosition;

import frc.robot.subsystems.DoubleArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition extends SequentialCommandGroup {
    
    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        addRequirements(doubleArm);
        addCommands(
            new SetDistalPositionInitial(doubleArm), 
            new SetProximalPosition(doubleArm, position[0]), 
            new SetDistalPosition(doubleArm, position[1])
        );
    }

}