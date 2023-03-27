package frc.robot.subsystems;

import frc.lib.util.PIDCalculator;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
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
import static frc.robot.Constants.GeneralConstants.*;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator; 
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private double last_time;
    private PIDCalculator pid;

    public Swerve() {
        pid = new PIDCalculator(pS, dS, iS, swerve_max_pid_rotation * Constants.BaseFalconSwerve.maxAngularVelocity, 180);
        gyro = new Pigeon2(Constants.BaseFalconSwerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro(180);

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

        poseEstimator = new SwerveDrivePoseEstimator(Constants.BaseFalconSwerve.swerveKinematics, Rotation2d.fromDegrees(180), getModulePositions(), new Pose2d(), VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.4, 0.4, 0.4)); 
    }

    /** Counterclockwise in degrees */
    public void changeHeading(double delta) {
        setTargetHeading(getYaw().getDegrees() + delta);
    }

    public void setTargetHeading(double target) {
        pid.reset(normalizeAngle(target - getYaw().getDegrees()) + getYaw().getDegrees());
        last_time = -999;
    }

    public void brake() {
        changeHeading(0);
        drive(new Translation2d(), 0, true, false);
    }

    public void makeX() {
        changeHeading(0);

        SwerveModuleState[] swerveModuleStates = { // FL, FR, BL, BR
            new SwerveModuleState(0.05, new Rotation2d(Math.PI / 4)), 
            new SwerveModuleState(0.05, new Rotation2d(-Math.PI / 4)), 
            new SwerveModuleState(0.05, new Rotation2d(3.0 * Math.PI / 4)), 
            new SwerveModuleState(0.05, new Rotation2d(-3.0 * Math.PI / 4))
        };
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.BaseFalconSwerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        double current_heading = getYaw().getDegrees();
        if (rotation == 0) {
            if (Timer.getFPGATimestamp() - last_time < swerve_calibration_time) {
                pid.reset(current_heading);
            } else {
                if (Math.abs(pid.getTarget() - current_heading) > 180) {
                    pid.reset(current_heading + normalizeAngle(pid.getTarget() - current_heading));
                }
                rotation = pid.update(current_heading);
            }
        } else {
            pid.reset(current_heading);
            last_time = Timer.getFPGATimestamp();
        }

        translation = translation.times(frc.robot.Variables.speed_factor);
        rotation *= frc.robot.Variables.speed_factor;

        if (translation.getNorm() < swerve_min_translation * Constants.BaseFalconSwerve.maxSpeed) translation = new Translation2d();
        if (Math.abs(rotation) < swerve_min_rotation * Constants.BaseFalconSwerve.maxAngularVelocity) rotation = 0;

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

    public void zeroGyro(double yaw) {
        gyro.setYaw(yaw);
        pid.reset(yaw);
        last_time = Timer.getFPGATimestamp();
    }

    public void zeroGyro() {
        poseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        gyro.setYaw(0);
        pid.reset(0);
        last_time = Timer.getFPGATimestamp();
    }

    public Rotation2d getYaw() {
        return (Constants.BaseFalconSwerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getError() {
        return normalizeAngle(pid.getTarget() - getYaw().getDegrees());
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
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

        Pose2d vision_estimate = Limelight.getRobotPosition();
        if (vision_estimate.getTranslation().getNorm() > 0.1 && (Math.abs(normalizeAngle(getYaw().getDegrees() - vision_estimate.getRotation().getDegrees())) < vision_min_error)) {
            poseEstimator.addVisionMeasurement(vision_estimate, Timer.getFPGATimestamp() - Limelight.getLatency());
            SmartDashboard.putString("Vision Pose", "(" + Math.round(vision_estimate.getTranslation().getX() * 100) / 100.0 + ", " + Math.round(vision_estimate.getTranslation().getY() * 100) / 100.0 + ")");
            SmartDashboard.putString("Vision Heading", "" + Math.round(vision_estimate.getRotation().getDegrees() * 100) / 100.0 + " degrees");
        } else if (vision_estimate.getTranslation().getNorm() > 0.1) {
            SmartDashboard.putString("Vision Pose", "Vision estimate did not make sense");
            SmartDashboard.putString("Vision Heading", "Vision estimate did not make sense");
        } else {
            SmartDashboard.putString("Vision Pose", "No vision estimate");
            SmartDashboard.putString("Vision Heading", "No vision estimate");
        }

        SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("Pitch ", getPitch());

        SmartDashboard.putString("Odometry Pose", "(" + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getX() * 100) / 100.0 + ", " + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getY() * 100) / 100.0 + ")");
        SmartDashboard.putString("Odometry Heading", "" + Math.round(poseEstimator.getEstimatedPosition().getRotation().getDegrees() * 100) / 100.0 + " degrees");
        
        // TODO: Make a visualizer in pygame
        
        double angle_current = 0;
        double drive_current = 0;

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            angle_current += mod.getCurrents()[0] / 4.0;
            drive_current += mod.getCurrents()[1] / 4.0;
        }

        SmartDashboard.putNumber("Average Angle Motor Current", angle_current);
        SmartDashboard.putNumber("Average Drive Motor Current", drive_current);
        SmartDashboard.putString("Side", Limelight.getSide());
    }
}