package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.TuningConstants.*;

public class Intake_Subcommand_One extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private DoubleArm doubleArm;

    public Intake_Subcommand_One(Claw claw, DoubleArm doubleArm) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it
    }

    @Override
    public void execute() {
        doubleArm.resetWhipControl();
        doubleArm.setTargetPositions(intakePosition);
        doubleArm.rawPowerArm(0, 0);
        SmartDashboard.putNumber("error", doubleArm.getTotalError());
    }
    
    @Override
    public boolean isFinished() {
        return doubleArm.getTotalError() < tolerance;
    }
}
