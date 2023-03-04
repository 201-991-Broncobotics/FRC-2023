package frc.robot.commands.setArmPosition;

import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.TuningConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition extends SequentialCommandGroup {
    
    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        addRequirements(doubleArm);
        addCommands(
            new SetMaxDistalPosition(doubleArm, intermediatePositionAngles[0]), 
            new SetProximalPosition(doubleArm, intermediatePositionAngles[0]), 
            new SetMaxDistalPosition(doubleArm, position[0]), 
            new SetProximalPosition(doubleArm, position[0]), 
            new SetDistalPosition(doubleArm, position[1])
        );
    }
}