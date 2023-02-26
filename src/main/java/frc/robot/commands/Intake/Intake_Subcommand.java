package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake_Subcommand extends CommandBase {
    
    private Claw claw;

    public Intake_Subcommand(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.intake();
    }

    @Override
    public void execute() {
        // nothing
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop();
    }
    
    @Override
    public boolean isFinished() {
        return (claw.getCurrent() > claw_current_limit);
    }
}
