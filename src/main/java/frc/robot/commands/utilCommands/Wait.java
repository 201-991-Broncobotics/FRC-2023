package frc.robot.commands.utilCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    
    private double starting_time, time;

    /** Wait time in seconds */
    public Wait(double time) {
        this.time = time;
    }

    @Override
    public void initialize() {
        starting_time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
    }
    
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - starting_time > time;
    }
    
}