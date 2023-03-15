package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

public class SetDistalPosition extends CommandBase { // small arm

    private DoubleArm doubleArm;
    private double distalPosition;

    public SetDistalPosition(DoubleArm doubleArm, double distalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.distalPosition = distalPosition;
    }

    @Override
    public void initialize() {
        doubleArm.setTargetAngles(new double[] {doubleArm.getTargetArmAngles()[0], distalPosition});
    }

    @Override
    public void execute() {
        doubleArm.pidPowerArm();
    }

    @Override
    public void end(boolean interrupted) {
        doubleArm.resetPID();
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(doubleArm.getCurrentArmAngles()[1] - distalPosition) < second_motor_tolerance;
    }

}
