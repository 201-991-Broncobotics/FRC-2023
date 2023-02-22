package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake_Subcommand extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Claw claw;
    private DoubleArm doubleArm;

    private boolean isFirstAction = true;

    public Intake_Subcommand(Claw claw, DoubleArm doubleArm) {
        this.doubleArm = doubleArm;
        this.claw = claw;
        addRequirements(claw);
        
        isFirstAction = true;
    }

    @Override
    public void execute() {
        if (isFirstAction) {
            claw.intake();
            isFirstAction = false;
            doubleArm.resetWhipControl();
            doubleArm.setTargetPositions(intakePosition);
        }
    }
    
    @Override
    public boolean isFinished() {
        if (claw.getCurrent() < claw_current_limit) {
            return false;
        } else {
            claw.stop();
            doubleArm.brake();
            doubleArm.setTargetPositions(idlePosition);
            isFirstAction = true;
            return true;
        }
    }
}
