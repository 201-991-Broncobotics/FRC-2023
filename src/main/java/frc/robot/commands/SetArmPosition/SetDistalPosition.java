package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetDistalPosition extends CommandBase { // small arm

    private DoubleArm doubleArm;
    private double distalPosition;

    public SetDistalPosition(DoubleArm doubleArm, double distalPosition) {
        this.doubleArm = doubleArm;
        // don't add reqs
        this.distalPosition = distalPosition;
    }

    @Override
    public void initialize() {
        doubleArm.resetWhipControl();
        doubleArm.setTargetPositions(DoubleArm.getPositionFromAngles(doubleArm.getTargetArmAngles()[0], distalPosition));
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
