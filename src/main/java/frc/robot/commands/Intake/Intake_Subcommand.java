package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.TuningConstants.*;

import java.util.function.BooleanSupplier;

public class Intake_Subcommand extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Claw claw;
    private DoubleArm doubleArm;

    private BooleanSupplier intakeSup;

    private double start_time;

    private boolean isFirstAction = true;

    public Intake_Subcommand(Claw claw, DoubleArm doubleArm, BooleanSupplier intakeSup) {
        this.doubleArm = doubleArm;
        this.claw = claw;
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it

        this.intakeSup = intakeSup;
        
        isFirstAction = true;
    }

    @Override
    public void execute() {
        if (isFirstAction) {
            start_time = System.currentTimeMillis() / 1000.0;
            claw.intake();
            isFirstAction = false;
            SmartDashboard.putNumber("it should be here", SmartDashboard.getNumber("it should be here", 0) + 1);
        }
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
