package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag extends CommandBase {

    private Swerve swerve;
    private double offset = 0.1;
    
    private double rotation_speed = 0.15;
    private double strafe_speed = 0.15;
    
    private double sideways_tolerance = 0.5 * 0.0254; // in m

    private BooleanSupplier stopSup;
    
    public AlignWithApriltag(Swerve swerve, BooleanSupplier stopSup) { // faster but less accurate
        this.swerve = swerve;
        addRequirements(swerve);

        this.stopSup = stopSup;
    }

    @Override
    public void execute() {
        
        // Filter out bad reads + reads that don't fit what the gyro should give (target should be between -5 to 5 mod 90)
        // Secondary filtering should be pretty easy, the initial filtering will be the hard part

        swerve.changeHeading(0);
        swerve.drive(new Translation2d(), 0, true, false); // brake

        double angular_offset = 0;
        for (int i = 0; i < 25; i++) {
            angular_offset += Limelight.getData()[1] / 25.0;
            if (Limelight.getData()[1] == -12) return;
            Timer.delay(0.01); // Limelight updates every 100hz
        }

        double target_heading = Swerve.normalizeAngle(swerve.getYaw().getDegrees() + angular_offset); // target pose = where we want to 

        while ((Math.abs(Swerve.normalizeAngle(target_heading - swerve.getYaw().getDegrees())) > 4) && (!stopSup.getAsBoolean())) {
            if (Swerve.normalizeAngle(target_heading - swerve.getYaw().getDegrees()) < 0) {
                swerve.drive(new Translation2d(), -rotation_speed * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            } else {
                swerve.drive(new Translation2d(), rotation_speed * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            }
        } // wait until we are good

        swerve.setTargetHeading(target_heading);

        while ((Limelight.getData()[2] == -12) && (!stopSup.getAsBoolean())) { // while we can't see the apriltag
            if (angular_offset < 0) {
                swerve.drive(new Translation2d(0, strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            } else {
                swerve.drive(new Translation2d(0, -strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            }
        }

        swerve.changeHeading(0);
        swerve.drive(new Translation2d(), 0, true, false); // brake

        double[] last_25_x = new double[25];
        double last_25_x_average = 0;
        int current_index = 0; // instead of reshuffling array each time

        for (int i = 0; i < 25; i++) {
            last_25_x[i] += Limelight.getData()[2] / 25.0;
            last_25_x_average += last_25_x[i];
            Timer.delay(0.01); // this also takes around 0.25 seconds to finish
        }

        swerve.setTargetHeading(target_heading);

        while ((Math.abs(offset - last_25_x_average) > sideways_tolerance) && (!stopSup.getAsBoolean()) && (Limelight.getData()[2] != -12)) {

            if (offset < last_25_x_average) {
                swerve.drive(new Translation2d(0, strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            } else {
                swerve.drive(new Translation2d(0, -strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            }

            last_25_x_average -= last_25_x[current_index];
            last_25_x[current_index] = Limelight.getData()[2] / last_25_x.length;
            last_25_x_average += last_25_x[current_index];
            current_index = (current_index + 1) % 25;
            Timer.delay(0.01);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
