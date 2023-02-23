package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.BaseFalconSwerve.angleEnableCurrentLimit, 
            Constants.BaseFalconSwerve.angleContinuousCurrentLimit, 
            Constants.BaseFalconSwerve.anglePeakCurrentLimit, 
            Constants.BaseFalconSwerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.BaseFalconSwerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.BaseFalconSwerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.BaseFalconSwerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.BaseFalconSwerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.BaseFalconSwerve.driveEnableCurrentLimit, 
            Constants.BaseFalconSwerve.driveContinuousCurrentLimit, 
            Constants.BaseFalconSwerve.drivePeakCurrentLimit, 
            Constants.BaseFalconSwerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.BaseFalconSwerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.BaseFalconSwerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.BaseFalconSwerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.BaseFalconSwerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.BaseFalconSwerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.BaseFalconSwerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.BaseFalconSwerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}