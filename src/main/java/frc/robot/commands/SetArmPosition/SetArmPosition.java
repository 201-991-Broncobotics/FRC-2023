package frc.robot.commands.setArmPosition;

import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition extends SequentialCommandGroup {
    
    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        addRequirements(doubleArm);
        addCommands(
            new SetMaxDistalPosition(doubleArm, Math.min(90, max_second_angle) - 180 + min_difference), 
            new SetProximalPosition(doubleArm, Math.min(90, max_second_angle) - 180 + min_difference), 
            new SetMaxDistalPosition(doubleArm, position[0]), 
            new SetProximalPosition(doubleArm, position[0]), 
            new SetDistalPosition(doubleArm, position[1])
        );
    }
}