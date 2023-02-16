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

    /* General Constants */
    public static final double joystick_deadzone = 0.2;

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
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder0_zero);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder1_zero);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder2_zero);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder3_zero);
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

        public static final int angle_trials = 25, 
                                distance_trials = 10; // longer means slower but more accurate

        public static final double offset = 0.1, // in inches
                                   sideways_tolerance = 0.5, // in inches
                                   strafe_speed = 0.15, 
                                   rotation_speed = 0.15, // percentage of maxima
                                   max_angular_tolerance = 999, // degrees
                                   angular_tolerance = 4;
    }

    public static final class SwerveConstants {
        
        public static final double maximum_power = 0.6, 
                                   maximum_error = 50, // degrees
                                   exponent = 0.5, 
                                   calibration_time = 0.5, // seconds
                                   tolerance = 2, // degrees
                                   correction = 0; // If we're going at full speed, how much do we need to correct by as a % - must be experimentally determined

            // best tuned so far: 0.6 as maximum_power, 50 as max_error, 0.5 as exponent
    }
    public static final class DoubleArmConstants {

        /* CAN IDs and Input Channels */

        public static final int first_motor_ID = 21, 
                                first_motor_follower_ID = 22, 
                                second_motor_ID = 23, // CAN IDs
                                first_encoder_channel = 1, 
                                second_encoder_channel = 0; // PWM Input Channels

        /* Motor Variables */
    
        public static final boolean invert_first_motor = true, 
                                    invert_first_motor_follower = false, 
                                    invert_second_motor = true, 
                                    invert_first_encoder = true, 
                                    invert_second_encoder = false, 
                                    first_motor_brake = true, 
                                    second_motor_brake = true;
        
        public static final int first_motor_max_current = 20, // amps
                                first_motor_follower_max_current = 20, 
                                second_motor_follower_max_current = 20;
        
        public static final double first_motor_acceleration_time = 3, 
                                   first_motor_voltage_compensation = 0, 
                                   second_motor_acceleration_time = 3, 
                                   second_motor_voltage_compensation = 0, 
                                   first_encoder_zero = TuningConstants.encoder_one_zero, 
                                   second_encoder_zero = TuningConstants.encoder_two_zero;

        /* Variables */
        
        public static final double arm_sensitivity = 10, // inches per second
                                   raw_arm_sensitivity = 0.35, // power ratio
                                   raw_arm_sensitivity_two = 0.15,

                                   min_x = -20, 
                                   min_y = -45, // inches

                                   clipping_one = 1.1, 
                                   clipping_two = 0.99, // first should be > 1, second should be < 1

                                   first_arm_length = 32, 
                                   second_arm_length = 20, // inches

                                   switching_angle = 0, // if below horizontal --> concave up, if above horizontal --> concave down

                                   first_motor_max_power = 0.5,
                                   second_motor_max_power = 0.2,

                                   first_motor_min_error = 1, 
                                   second_motor_min_error = 1,

                                   first_motor_max_error = 25.0, 
                                   second_motor_max_error = 25.0,

                                   k_exponent = 0.8,  // 1.0 for a normal PID

                                   tolerance = 3; // number of inches until we bing chilling
    }

    public static final class ClawConstants {

        /* CAN ID */

        public static final int claw_motor_ID = 24;

        /* Motor Variables */
    
        public static final boolean claw_motor_brake = true;
        
        public static final int claw_motor_max_current = 20;
        
        public static final double claw_motor_acceleration_time = 3, 
                                   claw_motor_voltage_compensation = 0;

        /* Variables */
        
        public static final double intake_power = 0.1, // one should be neg, one should be positive
                                   outtake_power = 0.1, 
                                   intake_time = 0.5, 
                                   outtake_time = 0.5; // seconds
    }

    public static final class Buttons {

        /* Driver Buttons */

        public static final int translationAxis = XboxController.Axis.kLeftY.value, 
                                strafeAxis = XboxController.Axis.kLeftX.value,
                                rotationAxis = XboxController.Axis.kRightX.value,
                                slowAxis = XboxController.Axis.kRightTrigger.value, 
                                zeroGyroButton = XboxController.Button.kY.value, 
                                robotCentricButton = XboxController.Button.kLeftBumper.value,
                                brakeButton = XboxController.Button.kX.value,
                                tagAlignerButton = XboxController.Button.kA.value,
                                tagAlignerExitButton = XboxController.Button.kA.value;

        /* Operator Buttons */

        public static final int horizAxis = XboxController.Axis.kLeftX.value, 
                                vertAxis = XboxController.Axis.kLeftY.value, 
                                motorOneAxis = XboxController.Axis.kLeftY.value, 
                                motorTwoAxis = XboxController.Axis.kRightY.value, 
                                topGoalButton = XboxController.Button.kY.value, 
                                midGoalButton = XboxController.Button.kB.value, 
                                lowGoalButton = XboxController.Button.kX.value, 
                                intakeButton = XboxController.Button.kA.value, 
                                outtakeButton = XboxController.Button.kLeftBumper.value, 
                                idleButton = XboxController.Button.kRightBumper.value, 
                                startPosButton = XboxController.Button.kStart.value;
    }

    public static final class TuningConstants {
        public static final double CANCoder0_zero = 291.09, 
                                   CANCoder1_zero = 297.86, 
                                   CANCoder2_zero = 320.18, 
                                   CANCoder3_zero = 295.57, 
                                   encoder_one_zero = -95.92, 
                                   encoder_two_zero = 178.1;

        // TODO: Tune these before EVERY SINGLE MATCH
        
        public static double[] topPosition = {44, 12}, 
                               midPosition = {40, 0}, 
                               lowPosition = {20, -20}, 
                               intakePosition = {20, -40}, 
                               idlePosition = {10, -10}, 
                               startPosition = {-1, -11};

        // TODO: still have to figure out these values lel

        public static boolean manual_control = true; // change this to true if we want to braindeadpower
    }
}
