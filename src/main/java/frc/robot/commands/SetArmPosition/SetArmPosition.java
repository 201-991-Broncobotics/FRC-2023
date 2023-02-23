package frc.robot.commands.SetArmPosition;

import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition extends SequentialCommandGroup {
    
    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        if (doubleArm.getTotalError() < tolerance) { // This might not work
            addRequirements(doubleArm); // means that other functions are not allowed to access it
            double[] target_angles = DoubleArm.getAnglesFromTarget(position[0], position[1]);
            if (doubleArm.getCurrentArmAngles()[1] < target_angles[1]) { // if our distal is below our target, set that first
                addCommands(
                    new SetDistalPosition(doubleArm, target_angles[1]), 
                    new SetProximalPosition(doubleArm, target_angles[0])
                );
            } else { // set proximal then distal
                addCommands(
                    new SetProximalPosition(doubleArm, target_angles[0]), 
                    new SetDistalPosition(doubleArm, target_angles[1])
                );
            }
        } else {
            addCommands(
                new InstantCommand()
            );
        }
    }
}
