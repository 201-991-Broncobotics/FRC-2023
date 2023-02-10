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
        if (Limelight.getData()[0] == -12) return;
        SmartDashboard.putNumberArray("yeah", Limelight.getData());
        SmartDashboard.putNumber("target turning", Math.max(Math.min(Limelight.getData()[1] * 0.1, 0.3), -0.3));
        SmartDashboard.putNumber("target strafing", Math.max(Math.min(Limelight.getData()[2] * 3, 0.3), -0.3));
        if (Math.abs(Limelight.getData()[1]) >= 3) { // turn, maximally if off by 30 degrees
            swerve.drive(new Translation2d(0, 0), Math.max(Math.min(Limelight.getData()[1] * 0.1, 0.3), -0.3) * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
        } else { // strafe, maximally at 10 cm away
            swerve.drive(new Translation2d(Math.max(Math.min(Limelight.getData()[2] * 3, 0.3), -0.3), 0).times(Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        }
    }

    @Override
    public boolean isFinished() {
        boolean returner = false;
        if (Limelight.getData()[0] == -12) {
            SmartDashboard.putNumber("detected tag", Limelight.getData()[0]);
            returner = true;
        } else {
            returner = ((Math.abs(Limelight.getData()[1]) < 3.5) && (Math.abs(Limelight.getData()[2]) < 0.025));
        }
        if (returner) { // what we do on last loop
            swerve.drive(new Translation2d(0, 0), 0, false, true);
        }
        return returner;
    }
}
