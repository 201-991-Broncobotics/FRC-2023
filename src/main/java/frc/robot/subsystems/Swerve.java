package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Variables;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.AprilTagAlignmentConstants.*; // for max angular tolerance

public class Swerve extends SubsystemBase {
    public static SwerveDrivePoseEstimator poseEstimator; 
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private double last_time;
    private double target_heading;

    public Swerve() {
        gyro = new Pigeon2(Constants.BaseFalconSwerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.BaseFalconSwerve.Mod0.constants),
            new SwerveModule(1, Constants.BaseFalconSwerve.Mod1.constants),
            new SwerveModule(2, Constants.BaseFalconSwerve.Mod2.constants),
            new SwerveModule(3, Constants.BaseFalconSwerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.BaseFalconSwerve.swerveKinematics, Rotation2d.fromDegrees(0), getModulePositions(), startingPose); 
    }

    /** Normalizes angle to between -180 and 180 */
    public static double normalizeAngle(double degrees) {
        if (degrees < 0) return ((degrees - 180) % 360 + 180);
        return ((degrees + 180) % 360 - 180);
    }

    /** Counterclockwise in degrees */
    public void changeHeading(double delta) {
        setTargetHeading(getYaw().getDegrees() + delta);
    }

    public void setTargetHeading(double target) {
        target_heading = normalizeAngle(target - getYaw().getDegrees()) + getYaw().getDegrees();
        last_time = System.currentTimeMillis() - (calibration_time + 1) * 1000;
    }

    public void brake() {
        changeHeading(0);
        drive(new Translation2d(), 0, true, false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        if (show_drive_data) {
            fieldRelative = false;
            translation = translation.times(0.33);
            rotation *= 0.33;
            Variables.data += "{" + 
                                (Math.round(translation.getX() * 1000.0) / 1000.0) + ", " + 
                                (Math.round(translation.getY() * 1000.0) / 1000.0) + ", " + 
                                (Math.round(rotation * 1000.0) / 1000.0) + ", " + 
                                (Math.round(Timer.getFPGATimestamp() * 1000.0) / 1000.0) + 
                              "}, "; // not println on purpose
                             // truncates to 3 decimals because more precision is not necessary
        }

        double current_heading = getYaw().getDegrees();
        if (rotation == 0) {
            if (System.currentTimeMillis() - last_time < calibration_time * 1000) {
                target_heading = current_heading;
            } else {
                if (Math.abs(target_heading - current_heading) > 180) {
                    target_heading = current_heading + normalizeAngle(target_heading - current_heading);
                }
                double error_in_percent = Math.max(Math.min((target_heading - current_heading) / maximum_error, 1), -1);
                int multiplier = 1;
                if (error_in_percent < 0) {
                    error_in_percent = 0 - error_in_percent;
                    multiplier = -1;
                }
                rotation = Math.pow(error_in_percent, exponent) * maximum_power * multiplier * Constants.BaseFalconSwerve.maxAngularVelocity;
            }
        } else {
            target_heading = current_heading;
            last_time = System.currentTimeMillis();
        }

        translation = translation.times(frc.robot.Variables.speed_factor);
        rotation *= frc.robot.Variables.speed_factor;

        SwerveModuleState[] swerveModuleStates =
            Constants.BaseFalconSwerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.BaseFalconSwerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.BaseFalconSwerve.maxSpeed);
        
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition(); 
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        target_heading = 0;
        last_time = System.currentTimeMillis();
    }

    public Rotation2d getYaw() {
        return (Constants.BaseFalconSwerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getError() {
        return normalizeAngle(target_heading - getYaw().getDegrees());
    }

    public double getPitch() {
        return gyro.getPitch();
    }
    
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModulePositions());

        double[] vision_estimate = Limelight.getRobotPosition();
        if (vision_estimate[1] != 0 && ((Math.abs(getYaw().getDegrees() - vision_estimate[2]) + max_angular_tolerance) % 90 < 2 * max_angular_tolerance)) {

            Pose2d estimatedPose = new Pose2d(vision_estimate[0], vision_estimate[1], new Rotation2d(vision_estimate[2] * Math.PI / 180));  // don't put yaw as the angle because yaw might be off by a few degrees
            addVisionEstimate(estimatedPose, Timer.getFPGATimestamp() - vision_estimate[3] * 1000.0);
            
        }

        SmartDashboard.putNumber("Gyro ", getYaw().getDegrees());
        SmartDashboard.putNumber("Pitch ", getPitch());

        SmartDashboard.putNumber("Estimated Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Estimated Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Estimated Pose Heading", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public static void addVisionEstimate(Pose2d estimate, double time) {
        poseEstimator.addVisionMeasurement(estimate, time);
    }
}