package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetDistalPosition extends CommandBase { // small arm

    private DoubleArm doubleArm;
    private double distalPosition;
    private boolean greater;

    public SetDistalPosition(DoubleArm doubleArm, double distalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.distalPosition = distalPosition;
    }

    @Override
    public void initialize() {
        doubleArm.setTargetAngles(new double[] {doubleArm.getTargetArmAngles()[0], distalPosition});
        greater = doubleArm.getCurrentArmAngles()[1] < distalPosition;
    }

    @Override
    public void execute() {
        doubleArm.bangbang(false);
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
        return (doubleArm.getTotalError() < tolerance) || ((doubleArm.getCurrentArmAngles()[1] > distalPosition) == greater);
    }

}
