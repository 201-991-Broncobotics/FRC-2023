package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends CommandBase {
    
    private Claw claw;

    private double starting_time;

    public Intake(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.intake();
        starting_time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // nothing
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop();
        if (!interrupted) {
            frc.robot.Variables.go_to_startposition = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return (claw.getCurrent() > claw_current_limit) && (Timer.getFPGATimestamp() - starting_time > 0.5);
    }
}
