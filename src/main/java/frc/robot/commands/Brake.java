package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Brake extends CommandBase {

    private Swerve swerve;
    
    public Brake(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        
        swerve.changeHeading(0);
        swerve.drive(new Translation2d(), 0, true, false); // brake
        Timer.delay(0.2);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
