package frc.robot.commands.Outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.IntakeConstants.*;

public class Outtake_Subcommand extends CommandBase {
    // Plan: we're already at outtake position, then outtake and reset target position for arm
    
    private Claw claw;

    private double starting_time;

    private boolean isFirstLoop = true;

    public Outtake_Subcommand(Claw claw) {
        this.claw = claw;
        addRequirements(claw); // means that other functions are not allowed to access it

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
            isFirstLoop = true;
            return true;
        }
        return false;
    }
}
