package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private double last_time;
    public double target_heading;

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

        swerveOdometry = new SwerveDriveOdometry(Constants.BaseFalconSwerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public static double normalizeAngle(double degrees) {
        if (degrees < 0) return ((degrees - 180) % 360 + 180);
        return ((degrees + 180) % 360 - 180);
    }

    public void changeHeading(double delta) {
        setTargetHeading(getYaw().getDegrees() + delta);
    }

    public void setTargetHeading(double target) {
        target_heading = normalizeAngle(target - getYaw().getDegrees()) + getYaw().getDegrees();
        last_time = System.currentTimeMillis() - (calibration_time + 1) * 1000;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        double current_heading = getYaw().getDegrees();
        if (rotation == 0) {
            if (System.currentTimeMillis() - last_time < calibration_time * 1000) {
                target_heading = current_heading;
            } else if (Math.abs(target_heading - current_heading) > tolerance) {
                double error_in_percent = Math.max(Math.min((target_heading - current_heading) / maximum_error, 1), -1);
                int multiplier = 1;
                if (error_in_percent < 0) {
                    error_in_percent = 0 - error_in_percent;
                    multiplier = -1;
                }
                rotation = Math.pow(error_in_percent, exponent) * maximum_power * multiplier * Constants.BaseFalconSwerve.maxAngularVelocity;
            } else {
                rotation = translation.getNorm() / Constants.BaseFalconSwerve.maxSpeed * Constants.BaseFalconSwerve.maxAngularVelocity * correction;
            }
        } else {
            target_heading = current_heading;
            last_time = System.currentTimeMillis();
        }

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

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.BaseFalconSwerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
        target_heading = 0;
        last_time = System.currentTimeMillis();
    }

    public Rotation2d getYaw() {
        return (Constants.BaseFalconSwerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }
    
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }

    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        SmartDashboard.putNumber("Gyro ", getYaw().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}