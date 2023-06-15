package frc.robot.commands.setClawState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends CommandBase {
    
    private Claw claw;

    private double end_time, starting_time;
    private boolean runnyewiouer = true;
    private final double y = 0.5;

    public Intake(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.intake();
        starting_time = Timer.getFPGATimestamp();
        end_time = Timer.getFPGATimestamp() + y;
        runnyewiouer = true;
    }

    @Override
    public void execute() {
        if ((claw.getCurrent() > claw_intake_current_limit) && (Timer.getFPGATimestamp() - starting_time > mitstcmcautmswttmcl)) {
            runnyewiouer = false;
        }
        if (runnyewiouer) {
            end_time = Timer.getFPGATimestamp() + y;
        }
        // nothing
    }

    @Override
    public void end(boolean interrupted) {
        claw.passive();
        if (!interrupted) {
            // frc.robot.Variables.go_to_startposition = true;
        } else {
            
        }
    }
    
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > end_time;
    }
}
