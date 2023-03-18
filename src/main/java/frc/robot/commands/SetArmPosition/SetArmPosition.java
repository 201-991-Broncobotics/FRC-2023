package frc.robot.commands.setArmPosition;

import frc.robot.subsystems.DoubleArm;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPosition extends SequentialCommandGroup {
    
    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        addRequirements(doubleArm);
        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetDistalPositionInitial(doubleArm), 
                    new SetProximalPosition(doubleArm, position[0]), 
                    new SetDistalPosition(doubleArm, position[1]), 
                    new InstantCommand(() -> doubleArm.resetPID())
                ), 
                new InstantCommand(), 
                () -> doubleArm.getUsingEncoders()
            )
        );
    }

}