package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetArmPosition extends CommandBase {

    private DoubleArm doubleArm;
    private double[] position;

    public SetArmPosition(DoubleArm doubleArm, double[] position) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.position = position;
    }

    @Override
    public void execute() {
        doubleArm.resetWhipControl();
        doubleArm.setTargetPositions(position);

        doubleArm.rawPowerArm(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return doubleArm.getTotalError() < tolerance;
    }
}