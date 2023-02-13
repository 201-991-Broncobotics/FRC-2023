package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class BaseFalconSwerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.00); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23.00); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(306.82);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(291.97);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(316.10);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(326.16);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AprilTagAlignmentConstants {
        public static final int tagAlignerButton = XboxController.Button.kA.value;

        public static final double offset = 0.1, // in inches
                                   sideways_tolerance = 0.5, // in inches
                                   strafe_speed = 0.15, 
                                   rotation_speed = 0.15; // percentage of maxima
        
        public static final double max_angular_tolerance = 10; // degrees

        public static final int angle_trials = 25, 
                                distance_trials = 10; // longer means slower but more accurate
    }

    public static final class SwerveConstants {
        public static final double joystick_deadzone = 0.2;
        public static final int translationAxis = XboxController.Axis.kLeftY.value, 
                          strafeAxis = XboxController.Axis.kLeftX.value,
                          rotationAxis = XboxController.Axis.kRightX.value,
                          slowAxis = XboxController.Axis.kRightTrigger.value, 
                          zeroGyroButton = XboxController.Button.kY.value, 
                          robotCentricButton = XboxController.Button.kLeftBumper.value,
                          brakeButton = XboxController.Button.kX.value;

        public static final double maximum_power = 0.6, 
                             maximum_error = 50, // degrees
                             exponent = 0.5, 
                             calibration_time = 0.5, // seconds
                             tolerance = 2, // degrees
                             correction = 0; // If we're going at full speed, how much do we need to correct by as a % - must be experimentally determined

            // best tuned so far: 0.6 as cappppping, 50 as max_error, 0.5 as exponent
    }
    public static final class DoubleArmConstants {

        // Controlling
        public static final double joystick_deadzone = 0.2;
        public static final int horizAxis = XboxController.Axis.kLeftX.value, 
                                vertAxis = XboxController.Axis.kLeftY.value, 
                                topGoalButton = XboxController.Button.kA.value, 
                                resetEncodersButton = XboxController.Button.kStart.value;

        public static final double arm_sensitivity = 5; // inches per second for arm

        public static final double min_x = 10, 
                                   min_y = -45; 
                                    // inches
        
        public static final double clipping_one = 1.1, 
                                   clipping_two = 0.9; // first should be > 1, second should be < 1
    
        public static final double first_arm_length = 32, 
                                   second_arm_length = 20; // inches
    
        public static final int first_motor_ID = 0, 
                                first_motor_follower_ID = 0, 
                                second_motor_ID = 0; // IDs
    
        public static final double switching_angle = -90; // always concave down
    
        public static final double first_motor_max_power = 1.0,
                                   second_motor_max_power = 1.0;
    
        public static final double first_motor_min_error = 0.5, 
                                   second_motor_min_error = 0.5;
    
        public static final double first_motor_max_error = 5.0, 
                                   second_motor_max_error = 5.0;
    
        public static final double k_exponent = 1.5; // 1.0 for a normal PID
    
        public static final int first_encoder_channel = 0, 
                                second_encoder_channel = 0;
    
        public static final boolean invert_first_motor = false, 
                                    invert_first_motor_follower = false, 
                                    invert_second_motor = false, 
                                    invert_first_encoder = false, 
                                    invert_second_encoder = false;
    }
}
