package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetMaxDistalPosition extends CommandBase { // big arm

    private DoubleArm doubleArm;
    
    private double distalPosition;

    public SetMaxDistalPosition(DoubleArm doubleArm, double[] target_angles) {
        this.doubleArm = doubleArm;

        distalPosition = Math.min(target_angles[0] + 180 - min_difference, target_angles[1]);
    }

    @Override
    public void initialize() { // we only want to run if our target proximal is above the current
        double target_distal = Math.min(
            Math.max(doubleArm.getCurrentArmAngles()[1], distalPosition), // whichever is greater of the 2 distal positions
            doubleArm.getCurrentArmAngles()[0] + 180 - min_difference // but don't set it to an illegal angle
        );
        doubleArm.resetWhipControl();
        doubleArm.setTargetAngles(new double[] {doubleArm.getCurrentArmAngles()[0], target_distal});
    }

    @Override
    public void execute() {
        doubleArm.powerArm(0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // code for if it ended by interruption
        } else {
            // code for if it ended by reaching the target position
            // if we manually move it it will say it's finished
            doubleArm.brake();
        }
    }
    
    @Override
    public boolean isFinished() {
        return doubleArm.getTotalError() < tolerance;
    }
}
