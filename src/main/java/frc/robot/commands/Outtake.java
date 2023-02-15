package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.ClawConstants.*;
import static frc.robot.Constants.DoubleArmConstants.*;

public class Outtake extends CommandBase {
    // Plan: we're already at outtake position, then outtake and reset target position for arm
    
    private Claw claw;
    private DoubleArm doubleArm;

    public Outtake(Claw claw, DoubleArm doubleArm) {
        this.doubleArm = doubleArm;
        this.claw = claw;
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it
    }

    @Override
    public void execute() {

        double time = System.currentTimeMillis() / 1000.0;
        claw.outtake();

        while (System.currentTimeMillis() / 1000.0 - time < intake_time) {
            doubleArm.moveArm(0, 0);
        }
        
        claw.stop();
        doubleArm.setTargetPositions(idlePosition);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
