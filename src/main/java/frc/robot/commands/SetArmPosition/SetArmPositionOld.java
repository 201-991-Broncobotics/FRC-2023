package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetArmPositionOld extends CommandBase {

    private DoubleArm doubleArm;
    private double[] position;

    public SetArmPositionOld(DoubleArm doubleArm, double[] position) {
        this.doubleArm = doubleArm;

        this.position = position;
    }

    @Override
    public void initialize() {
        doubleArm.resetWhipControl();
        doubleArm.setTargetPositions(position);
    }

    @Override
    public void execute() {
        doubleArm.rawPowerArm(0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {

        } else {
            doubleArm.brake();
        }
    }
    
    @Override
    public boolean isFinished() {
        return doubleArm.getTotalError() < tolerance;
    }
}