package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag extends CommandBase {

    private Swerve swerve;
    
    public AlignWithApriltag(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (Limelight.getData()[0] == -12) return;
        if (Math.abs(Limelight.getData()[1]) >= 5) { // turn
            swerve.drive(new Translation2d(0, 0), Math.max(Math.min(Limelight.getData()[1] * 0.1, 0.3), -0.3), false, false);
        } else { // strafe
            swerve.drive(new Translation2d(Math.max(Math.min(Limelight.getData()[2] * 0.1, 0.3), -0.3), 0), 0, false, false);
        }
    }

    @Override
    public boolean isFinished() {
        if (Limelight.getData()[0] == -12) {
            SmartDashboard.putNumber("jhasfdashkj", 1);
            return true;
        }
        return ((Math.abs(Limelight.getData()[1]) < 5) && (Math.abs(Limelight.getData()[2]) < 0.05));
    }
}
