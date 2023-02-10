package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        while (Limelight.getData()[1] > 1 && Limelight.getData()[0] != -12) {
            if (Limelight.getData()[2] + 0.1 > 0.01) {
                swerve.drive(new Translation2d(0, -0.1 * Constants.BaseFalconSwerve.maxSpeed), -0.05 * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            } else if (Limelight.getData()[2] + 0.1 < -0.01) {
                swerve.drive(new Translation2d(0, 0.1 * Constants.BaseFalconSwerve.maxSpeed), -0.05 * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            } else {
                swerve.drive(new Translation2d(), -0.05 * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            }
        }
        while (Limelight.getData()[1] < -1 && Limelight.getData()[0] != -12) {
            if (Limelight.getData()[2] + 0.1 > 0.01) {
                swerve.drive(new Translation2d(0, -0.1 * Constants.BaseFalconSwerve.maxSpeed), 0.05 * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            } else if (Limelight.getData()[2] + 0.1 < -0.01) {
                swerve.drive(new Translation2d(0, 0.1 * Constants.BaseFalconSwerve.maxSpeed), 0.05 * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            } else {
                swerve.drive(new Translation2d(), 0.05 * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            }
        }
        while (Limelight.getData()[2] + 0.1 > 0.01 && Limelight.getData()[0] != -12) {
            swerve.drive(new Translation2d(0, -0.1 * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        }
        
        while (Limelight.getData()[2] + 0.1 < -0.01 && Limelight.getData()[0] != -12) {
            swerve.drive(new Translation2d(0, 0.1 * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
