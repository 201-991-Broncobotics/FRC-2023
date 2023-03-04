package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetMaxDistalPosition extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double distalPosition;
    private boolean greater;
    private double target_distal;

    public SetMaxDistalPosition(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        distalPosition = proximalPosition + 180 - min_difference;
    }

    @Override
    public void initialize() { // we only want to run if our target proximal is above the current
        target_distal = Math.min(
            doubleArm.getCurrentArmAngles()[0] + 180 - min_difference, 
            distalPosition
        ); // maximal possible distal position

        doubleArm.setTargetAngles(new double[] {doubleArm.getCurrentArmAngles()[0], target_distal});
        greater = doubleArm.getCurrentArmAngles()[1] < target_distal;
    }

    @Override
    public void execute() {
        doubleArm.bangbang(false);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            doubleArm.resetPID();
        }
    }
    
    @Override
    public boolean isFinished() {
        return (doubleArm.getTotalError() < tolerance) || ((doubleArm.getCurrentArmAngles()[1] > target_distal) == greater);
    }
}
