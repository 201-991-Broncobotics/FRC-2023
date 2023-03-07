package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetDistalPositionInitial extends CommandBase { // small arm

    private DoubleArm doubleArm;
    private boolean greater;
    private double targetDistal;

    public SetDistalPositionInitial(DoubleArm doubleArm) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);
    }

    @Override
    public void initialize() {
        doubleArm.resetPID();
        targetDistal = Math.min(Math.min(90, second_motor_max_angle), doubleArm.getTargetArmAngles()[0] + 180 - min_difference);
        doubleArm.setTargetAngles(new double[] {doubleArm.getTargetArmAngles()[0], targetDistal});
        greater = doubleArm.getCurrentArmAngles()[1] < targetDistal;
    }

    @Override
    public void execute() {
        doubleArm.pidPowerArm();;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            doubleArm.resetPID();
        }
    }
    
    @Override
    public boolean isFinished() {
        return (doubleArm.getCurrentArmAngles()[1] > targetDistal) == greater;
    }

}