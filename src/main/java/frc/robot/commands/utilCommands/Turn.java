package frc.robot.commands.utilCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.SwerveConstants.*;

public class Turn extends CommandBase {
        
    private Swerve swerve;
    private double amount;

    public Turn(Swerve swerve, double amount) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.amount = amount;
    }

    @Override
    public void initialize() {
        swerve.changeHeading(amount);
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(), 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getError()) < yaw_tolerance;
    }
    
}