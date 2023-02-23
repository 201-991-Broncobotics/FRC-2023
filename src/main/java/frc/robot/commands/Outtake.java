package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.IntakeConstants.*;

public class Outtake extends CommandBase {
    // Plan: we're already at outtake position, then outtake and reset target position for arm
    
    private Claw claw;
    private DoubleArm doubleArm;

    private double starting_time;

    private boolean isFirstLoop = true;

    public Outtake(Claw claw, DoubleArm doubleArm) {
        this.doubleArm = doubleArm;
        this.claw = claw;
        addRequirements(doubleArm, claw); // means that other functions are not allowed to access it

        isFirstLoop = true;
        starting_time = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        if (isFirstLoop) {
            claw.outtake();
            starting_time = System.currentTimeMillis() / 1000.0;
            isFirstLoop = false;
        }
    }
    
    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() / 1000.0 - starting_time > outtake_time) {
            claw.stop();
            doubleArm.brake();
            isFirstLoop = true;
            return true;
        }
        return false;
    }
}
