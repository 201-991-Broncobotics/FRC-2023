package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetDistalPositionOne extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double distalPosition;

    public SetDistalPositionOne(DoubleArm doubleArm, double distalPosition) {
        this.doubleArm = doubleArm;

        this.distalPosition = distalPosition;
    }

    @Override
    public void initialize() { // we only want to run if our target proximal is above the current
        if (distalPosition > doubleArm.getCurrentArmAngles()[1]) {
            doubleArm.resetWhipControl();
            doubleArm.setTargetPositions(DoubleArm.getPositionFromAngles(doubleArm.getTargetArmAngles()[0], distalPosition));
        } else {
            doubleArm.setTargetPositions(doubleArm.getCurrentXY());
        }
    }

    @Override
    public void execute() {
        doubleArm.rawPowerArm(0, 0);
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
