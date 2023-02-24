package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetDistalPosition extends CommandBase { // small arm

    private DoubleArm doubleArm;
    private double[] targetPosition;

    private boolean isFirstLoop = true;

    public SetDistalPosition(DoubleArm doubleArm, double distalPosition) {
        this.doubleArm = doubleArm;
        // addRequirements(doubleArm);

        this.targetPosition = DoubleArm.getPositionFromAngles(doubleArm.getTargetArmAngles()[0], distalPosition);
        isFirstLoop = true;
    }

    @Override
    public void execute() {
        if (isFirstLoop) {
            doubleArm.resetWhipControl();
            doubleArm.setTargetPositions(targetPosition);
            isFirstLoop = false;
        }

        doubleArm.rawPowerArm(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        if (doubleArm.getTotalError() < tolerance) {
            doubleArm.brake();
            isFirstLoop = true;
            return true;
        }
        return false;
    }

}
