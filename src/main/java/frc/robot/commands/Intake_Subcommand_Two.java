package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.ClawConstants.*;
import static frc.robot.Constants.TuningConstants.*;

import java.util.function.BooleanSupplier;

public class Intake_Subcommand_Two extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Claw claw;
    private DoubleArm doubleArm;

    private BooleanSupplier intakeSup;

    private double start_time;

    public Intake_Subcommand_Two(Claw claw, DoubleArm doubleArm, BooleanSupplier intakeSup) {
        this.doubleArm = doubleArm;
        this.claw = claw;
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it

        this.intakeSup = intakeSup;
        
        doubleArm.setTargetPositions(intakePosition);
        start_time = System.currentTimeMillis() / 1000.0;
        claw.intake();
    }

    @Override
    public void execute() {
        doubleArm.resetWhipControl();
        doubleArm.setTargetPositions(intakePosition);
        doubleArm.rawPowerArm(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        if ((System.currentTimeMillis() / 1000.0 - start_time < intake_time) || intakeSup.getAsBoolean()) {
            return false;
        } else {
            claw.stop();
            doubleArm.brake();
            doubleArm.setTargetPositions(idlePosition);
            return true;
        }
    }
}
