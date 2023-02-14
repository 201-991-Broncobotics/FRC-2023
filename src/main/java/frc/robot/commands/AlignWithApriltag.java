package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AprilTagAlignmentConstants.*;

public class AlignWithApriltag extends CommandBase {

    private Swerve swerve;

    private XboxController driver;
    
    public AlignWithApriltag(Swerve swerve, XboxController driver) { // faster but less accurate
        this.swerve = swerve;
        addRequirements(swerve);

        this.driver = driver;
    }

    @Override
    public void execute() {
        
        // Filter out bad reads + reads that don't fit what the gyro should give (target should be between -5 to 5 mod 90)
        // Secondary filtering should be pretty easy, the initial filtering will be the hard part

        swerve.changeHeading(0);
        swerve.drive(new Translation2d(), 0, true, false); // brake

        double angular_offset = 0;
        for (int i = 0; i < angle_trials; i++) {
            if (Limelight.getData()[1] == -12) return;
            if ((Math.abs(swerve.getYaw().getDegrees() + Limelight.getData()[1]) + max_angular_tolerance) % 90 > 2 * max_angular_tolerance) { // this is the target angle
                i--;
            } else {
                angular_offset += Limelight.getData()[1] / (1.0 * angle_trials);
            }
            Timer.delay(0.01); // Limelight updates every 100hz
        }

        System.out.println(swerve.getYaw().getDegrees());
        System.out.println(angular_offset);
        System.out.println((Math.abs(swerve.getYaw().getDegrees() + angular_offset) + max_angular_tolerance) % 90);

        double target_heading = Swerve.normalizeAngle(swerve.getYaw().getDegrees() + angular_offset); // target pose = where we want to 

        while ((Math.abs(Swerve.normalizeAngle(target_heading - swerve.getYaw().getDegrees())) > angular_tolerance) && (!driver.getRawButton(tagAlignerExitButton))) {
            if (Swerve.normalizeAngle(target_heading - swerve.getYaw().getDegrees()) < 0) {
                swerve.drive(new Translation2d(), -rotation_speed * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            } else {
                swerve.drive(new Translation2d(), rotation_speed * Constants.BaseFalconSwerve.maxAngularVelocity, false, true);
            }
        } // wait until we are good

        swerve.setTargetHeading(target_heading);

        while ((Limelight.getData()[2] == -12) && (!driver.getRawButton(tagAlignerExitButton))) { // while we can't see the apriltag
            if (angular_offset < 0) {
                swerve.drive(new Translation2d(0, strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            } else {
                swerve.drive(new Translation2d(0, -strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            }
        }

        if (angular_offset < 0) {
            swerve.drive(new Translation2d(0, strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, false);
        } else {
            swerve.drive(new Translation2d(0, -strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, false);
        }
        
        Timer.delay(0.3);

        swerve.changeHeading(0);
        swerve.drive(new Translation2d(), 0, true, false); // brake

        System.out.println("Ok, we got here successfully");

        double[] prev_x = new double[distance_trials];
        double prev_x_average = 0;
        int current_index = 0; // instead of reshuffling array each time

        for (int i = 0; i < distance_trials; i++) {
            if (Math.abs(Limelight.getData()[1]) > max_angular_tolerance) {
                i--;
            } else {
                prev_x[i] += Limelight.getData()[2] / (1.0 * distance_trials);
                prev_x_average += prev_x[i];
            }
            Timer.delay(0.01);
        }

        swerve.setTargetHeading(target_heading);

        while ((Math.abs(offset * 0.0254 - prev_x_average) > sideways_tolerance * 0.0254) && (!driver.getRawButton(tagAlignerExitButton)) && (Limelight.getData()[2] != -12)) {

            if (offset * 0.0254 < prev_x_average) {
                swerve.drive(new Translation2d(0, strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            } else {
                swerve.drive(new Translation2d(0, -strafe_speed * Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
            }

            prev_x_average -= prev_x[current_index];
            if (Math.abs(Limelight.getData()[1]) > max_angular_tolerance) {
                // guess based on previous two if our thing doesn't make sense
                prev_x[current_index] = 2 * prev_x[(current_index + distance_trials - 1) % distance_trials] - prev_x[(current_index + distance_trials - 2) % distance_trials];
            } else {
                prev_x[current_index] = Limelight.getData()[2] / (1.0 * distance_trials);
            }
            prev_x_average += prev_x[current_index];
            current_index = (current_index + 1) % distance_trials;
            Timer.delay(0.01);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
