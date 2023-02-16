package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.ClawConstants.*;
import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.TuningConstants.*;

import static frc.robot.Constants.Buttons.*;

public class Intake extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Claw claw;
    private DoubleArm doubleArm;

    public Intake(Claw claw, DoubleArm doubleArm) {
        this.doubleArm = doubleArm;
        this.claw = claw;
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it
    }

    @Override
    public void execute() {
        doubleArm.setTargetPositions(intakePosition);

        while (doubleArm.getTotalError() > tolerance) {
            doubleArm.rawPowerArm(0, 0); // it really doesn't matter
        }

        double time = System.currentTimeMillis() / 1000.0;
        claw.intake();

        while ((System.currentTimeMillis() / 1000.0 - time < intake_time)) {
            doubleArm.moveArm(0, 0);
        }
        
        claw.stop();
        doubleArm.brake();
        doubleArm.setTargetPositions(idlePosition);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
