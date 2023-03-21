package frc.robot.commands.autoBalance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.SwerveConstants.*;

public class AutoBalance_Subcommand_1 extends CommandBase {
    
    private Swerve swerve;

    public AutoBalance_Subcommand_1(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setTargetHeading(((Math.abs(swerve.getYaw().getDegrees()) + 90) % 360 < 180) ? 0 : 180);
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
