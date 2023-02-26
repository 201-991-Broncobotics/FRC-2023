package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.IntakeConstants.*;

public class Outtake_Subcommand extends CommandBase {
    // Plan: we're already at outtake position, then outtake and reset target position for arm
    
    private Claw claw;
    private double starting_time;

    public Outtake_Subcommand(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.outtake();
        starting_time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // don't do anything
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop();
    }
    
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - starting_time > outtake_time;
    }
}
