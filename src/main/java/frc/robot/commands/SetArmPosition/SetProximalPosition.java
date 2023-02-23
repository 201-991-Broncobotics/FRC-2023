package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetProximalPosition extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double[] targetPosition;

    private boolean isFirstLoop = true;

    public SetProximalPosition(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.targetPosition = DoubleArm.getPositionFromAngles(proximalPosition, doubleArm.getTargetArmAngles()[1]);
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
