package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetArmPosition extends CommandBase {

    private DoubleArm doubleArm;
    private double[] position;

    private boolean isFirstLoop = true;

    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.position = position;
        isFirstLoop = true;
    }

    @Override
    public void execute() {
        if (isFirstLoop) {
            doubleArm.resetWhipControl();
            doubleArm.setTargetPositions(position);
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