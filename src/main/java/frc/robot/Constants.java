package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class BaseFalconSwerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.750); // May have to change for future robots
        public static final double wheelBase = Units.inchesToMeters(22.750);

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

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Change the values above if you want to but they're not necessary to be tuned */

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
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

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = TuningConstants.drive_motor_p;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (TuningConstants.drive_static_voltage / 12.0);
        public static final double driveKV = (TuningConstants.drive_equilibrium_voltage / 12.0);
        public static final double driveKA = (TuningConstants.drive_acceleration_voltage / 12.0);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = TuningConstants.max_linear_speed * 0.3048;
        /** Radians per Second */
        public static final double maxAngularVelocity = TuningConstants.max_angular_speed * Math.PI / 180.0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder0_zero);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder1_zero);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder2_zero);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder3_zero);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = TuningConstants.max_linear_speed_autonomous * 0.3048;
        public static final double kMaxAccelerationMetersPerSecondSquared = TuningConstants.max_linear_speed_autonomous * 0.3048 / TuningConstants.ramp_up_time_linear;
        public static final double kMaxAngularSpeedRadiansPerSecond = TuningConstants.max_angular_speed_autonomous * Math.PI / 180.0;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = TuningConstants.max_angular_speed_autonomous * Math.PI / 180.0 / TuningConstants.ramp_up_time_angular;
    
        public static final double kPXController = TuningConstants.translation_p_controller;
        public static final double kPYController = TuningConstants.translation_p_controller;
        public static final double kPThetaController = TuningConstants.angle_p_controller;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, 
            kMaxAngularSpeedRadiansPerSecondSquared
        );
    }

    public static final class AprilTagAlignmentConstants {

        public static final int angle_trials = 25, 
                                distance_trials = 10; // longer means slower but more accurate

        public static final double offset = 0.1, // in inches; from center of robot
                                   sideways_tolerance = 0.5, // in inches
                                   sideways_speed = 0.15, 
                                   rotation_speed = 0.15, // percentage of maxima
                                   max_angular_tolerance = 999, // degrees
                                   angular_tolerance = 15,
                                   extra_time_to_be_in_frame = 0.25, 
                                   max_calculation_time = 1, 
                                   max_time_to_get_in_frame = 3,
                                   max_alignment_time = 5;
    }

    public static final class AutoBalanceConstants {
        public static final double drive_speed = 0.16,
                                   pitch_tolerance = 8, 
                                   max_linear_acceleration = 0.4,
                                   ratio = 1.38, 
                                   min_time = 1.5, 
                                   min_deriv = 1.5;
    }

    public static final class ClawConstants {

        /* CAN ID */

        public static final int claw_motor_ID = 24;

        /* Motor Variables */
    
        public static final boolean claw_motor_brake = true;
        
        public static final int claw_motor_max_current = 30;
        
        public static final double claw_motor_acceleration_time = 3, 
                                   claw_motor_voltage_compensation = 0;

        /* Variables */
        
        public static final double intake_power = -1, // intake with negative power
                                   outtake_power = 1;
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
                                    invert_second_encoder = true, 
                                    first_motor_brake = true, 
                                    second_motor_brake = true;
        
        public static final int first_motor_max_current = 20, // amps
                                first_motor_follower_max_current = 20, 
                                second_motor_max_current = 20;
        
        public static final double first_motor_acceleration_time = 3, 
                                   first_motor_voltage_compensation = 0, 
                                   second_motor_acceleration_time = 3, 
                                   second_motor_voltage_compensation = 0, 
                                   first_encoder_zero = TuningConstants.encoder_one_zero, 
                                   second_encoder_zero = TuningConstants.encoder_two_zero;

        /* Variables */
        
        public static final double first_motor_sensitivity = 0.55,
                                   first_motor_max_power = 0.6,

                                   second_motor_sensitivity = 0.35,
                                   second_motor_max_power = 0.4,

                                   first_motor_max_acceleration = 0.6, // we can't change this as fast
                                   second_motor_max_acceleration = 1.2, // we can change this very fast

                                   first_motor_min_error = 0.5, 
                                   second_motor_min_error = 0.5,

                                   first_motor_max_error = 55.0, 
                                   second_motor_max_error = 55.0,

                                   whiplash_time_one = 1.25, 
                                   whiplash_time_two = 0.4, 

                                   k_exponent = 1.15,  // 1.0 for a normal PID

                                   tolerance = 3, // number of inches until we bing chilling

                                   first_arm_length = 32 - 4, // subtract 4 from length of arm to get pivot distance 
                                   second_arm_length = 15 + 7.751984, // add 7.751984 to length of arm to get pivot distance

                                   min_x = 4, 
                                   min_y = -42,
                                   max_x = 61.368, 
                                   max_y = 20, 

                                   min_first_angle = -115, 
                                   max_first_angle = 20, 
                                   min_second_angle = -80, 
                                   max_second_angle = 80, 
                                   min_difference = 20, 

                                   clipping_one = 1.2, 
                                   clipping_two = 0.99,

                                   switching_angle = 90, // if below this --> concave up, if above this --> concave down

                                   middle_x = 45, // above this, we go at a slower rate
                                   middle_y = 0, 
                                   speed_when_arm_extended = 0.35;
    }
    
    public static final class IntakeConstants {
        public static final double claw_current_limit = 25, 
                                   outtake_time = 0.5;
    }

    public static final class SwerveConstants {
        
        public static final double maximum_power = 0.8, 
                                   maximum_error = 60, // degrees
                                   exponent = 0.85, 
                                   calibration_time = 0.5, // seconds
                                   turn_sensitivity = 0.35, 
                                   slow = 0.35,
                                   yaw_tolerance = 2;
    }

    public static final class AutonomousNames {
        public static final String[][] autos = {
            {"Default", "Determine Path from Limelight"}, // first one is the default
            {"Red Left", "Run Path for Tag 1"},
            {"Red Center", "Run Path for Tag 2"},
            {"Red Right", "Run Path for Tag 3"},
            {"Blue Left", "Run Path for Tag 6"},
            {"Blue Center", "Run Path for Tag 7"},
            {"Blue Right", "Run Path for Tag 8"},
            {"Place Cube", "Run Path for Tag"},
            {"Test One", "Drive Forward One Meter"},
            {"Test Two", "Strafe Right One Meter"},
            {"Test Three", "Turn Clockwise 90 degrees"}, 
            {"Test Path", "Run the Test Path"}
        };
    }
    public static final class Buttons {

        /* Which is which */

        public static final int driver_usb_port = 1, 
                                operator_usb_port = 0;

        /* Driver Buttons */

        public static final int translationAxis = XboxController.Axis.kLeftY.value, 
                                strafeAxis = XboxController.Axis.kLeftX.value,
                                rotationAxis = XboxController.Axis.kRightX.value,
                                slowAxis = XboxController.Axis.kRightTrigger.value, 

                                directionXAxis = XboxController.Axis.kRightX.value, 
                                directionYAxis = XboxController.Axis.kRightY.value, 
                                turnRightAxis = XboxController.Axis.kRightTrigger.value, 
                                turnLeftAxis = XboxController.Axis.kLeftTrigger.value, 
                                slowButtonOne = XboxController.Button.kRightBumper.value, 
                                slowButtonTwo = XboxController.Button.kLeftBumper.value, 

                                zeroGyroButton = XboxController.Button.kY.value, 
                                robotCentricButton = XboxController.Button.kLeftBumper.value,
                                tagAlignerButton = XboxController.Button.kA.value,
                                tagAlignerExitButton = XboxController.Button.kA.value, 
                                tagAlignerNewButton = XboxController.Button.kX.value, 
                                autoBalanceButton = XboxController.Button.kB.value,
                                terminateCommandsDriverButton = XboxController.Button.kBack.value;

        /* Operator Buttons */

        public static final int motorOneAxis = XboxController.Axis.kLeftY.value, 
                                motorTwoAxis = XboxController.Axis.kRightY.value, 

                                topGoalButton = XboxController.Button.kY.value, 
                                midGoalButton = XboxController.Button.kB.value, 
                                lowGoalButton = XboxController.Button.kA.value, 

                                intakeButton = XboxController.Button.kRightBumper.value, 
                                outtakeButton = XboxController.Button.kLeftBumper.value, 

                                idleButton = XboxController.Button.kX.value, 
                                startPosButton = XboxController.Button.kStart.value,

                                stopArmFromMovingButtonOne = XboxController.Axis.kRightTrigger.value,
                                stopArmFromMovingButtonTwo = XboxController.Axis.kLeftTrigger.value,
                                terminateCommandsOperatorButton = XboxController.Button.kBack.value;
            
        /* General Constants */

        public static final double joystick_deadzone = 0.1,
                                   axis_exponent = 1.3;

        public static final double signedPower(double axis_value) {
            axis_value = MathUtil.applyDeadband(axis_value, joystick_deadzone);
            if (axis_value == 0) return 0;
            if (axis_value < 0) return 0 - Math.pow(0 - axis_value, axis_exponent);
            return Math.pow(axis_value, axis_exponent);
        }
    }

    public static final class TuningConstants {
        public static final double CANCoder0_zero = 277.38, 
                                   CANCoder1_zero = 292.5, 
                                   CANCoder2_zero = 318.78, 
                                   CANCoder3_zero = 271.14, 
                                   encoder_one_zero = -90.3, 
                                   encoder_two_zero = -301.6;
        
        /* The angles we want to go to - NOT the x and y */
        public static final double[] topPositionAngles = {2.7, 19.0}, 
                                     midPositionAngles = {-21.4, 0}, 
                                     lowPositionAngles = {-64, 0}, 
                                     intakePositionAngles = {-82.3, -32.9}, 
                                     idlePositionAngles = {-90, 60}, 
                                     startPositionAngles = {-115, 50}, 
                                     
                                     intermediatePositionAngles = {-70, 80};

        public static final boolean fancy_drive = true, 
                                    show_drive_data = false, 
                                    test_autonomous = true; // runs TestPath - should be somethings simple like turning or driving straight
        
        /* Swerve Drive Constants */

        public static final double drive_motor_p = 0.05,
                                   drive_static_voltage = 0.32, 
                                   drive_equilibrium_voltage = 1.51, 
                                   drive_acceleration_voltage = 0.27, // SYSID values: KS, KV, KA; they are automatically divided by 12 later
                                   max_linear_speed = 10, // feet per second
                                   max_angular_speed = 180; // degrees per second

        /* Auto Constants */

        public static final double max_linear_speed_autonomous = 5,
                                   ramp_up_time_linear = 2, // in seconds to reach max 
                                   max_angular_speed_autonomous = 180, 
                                   ramp_up_time_angular = 2,
                                   translation_p_controller = 2, 
                                   angle_p_controller = 4;
    }
}
