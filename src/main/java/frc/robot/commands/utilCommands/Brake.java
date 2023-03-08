package frc.robot.commands.utilCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Brake extends CommandBase {
        
    private Swerve swerve;
    private double end_time;

    public Brake(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.changeHeading(0);
        end_time = Timer.getFPGATimestamp() + 0.5;
    }

    @Override
    public void execute() {
        swerve.makeX();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > end_time;
    }
    
}