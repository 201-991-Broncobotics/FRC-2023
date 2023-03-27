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
        public static final int angleContinuousCurrentLimit = 10;
        public static final int anglePeakCurrentLimit = 20;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 15;
        public static final int drivePeakCurrentLimit = 20;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

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

    public static final class AutoBalanceConstants {
        public static final double drive_speed = 0.1,
                                   drive_speed_get_on = 0.2, 
                                   pitch_tolerance = 7, 
                                   max_linear_acceleration = 2.0,
                                   ratio = 1.15, 
                                   min_time = 0.5, 
                                   mt2 = 2.2, 
                                   min_deriv = 0.025;
    }

    public static final class ClawConstants {

        /* CAN ID */

        public static final int claw_motor_ID = 24;

        /* Motor Variables */
    
        public static final boolean claw_motor_brake = true;
        
        public static final int claw_motor_max_current = 30;

        /* Variables */
        
        public static final double intake_power = -1, // intake with negative power
                                   outtake_power = 0.3,
                                   passive_voltage = -1.5;
    }

    public static final class DoubleArmConstants {

        /* CAN IDs and Input Channels */

        public static final int proximal_id = 21, 
                                proximal_follower_id = 22, 
                                distal_id = 23, // CAN IDs

                                first_encoder_channel = 1, 
                                second_encoder_channel = 0; // PWM Input Channels

        /* Motor Variables */
    
        public static final boolean invert_proximal = true, 
                                    invert_distal = true, 
                                    invert_proximal_encoder = false, 
                                    invert_distal_encoder = false, 
                                    invert_first_encoder = true, 
                                    invert_second_encoder = true, 
                                    proximal_brake = true, 
                                    distal_brake = true;
        
        public static final int proximal_max_continuous_current = 15, // amps
                                proximal_max_peak_current = 20, 
                                distal_max_continuous_current = 15, 
                                distal_max_peak_current = 20, 
                                second_motor_stop_current = 19;
        
        public static final double proximal_peak_current_time = 0.1, 
                                   proximal_open_ramp_rate = 0.1, 
                                   proximal_closed_ramp_rate = 0.1, 
                                   distal_peak_current_time = 0.1, 
                                   distal_open_ramp_rate = 0.1, 
                                   distal_closed_ramp_rate = 0.1, 
                                   first_encoder_zero = TuningConstants.encoder_one_zero, 
                                   second_encoder_zero = TuningConstants.encoder_two_zero;

        /* Variables */
        
        public static final double first_motor_sensitivity = 0.55,

                                   proximal_max_percentoutput = 0.6, 

                                   proximal_max_percentoutput_per_second = 2.0,
                                   proximal_calibration_time = 0.5, 

                                   proximal_min_angle = -115, 
                                   proximal_max_angle = 20, 

                                   proximal_gear_ratio = 1224.0 / 7.0 * 1.2, 

                                   first_motor_tolerance = 4, 



                                   second_motor_sensitivity = 0.75,

                                   distal_max_percentoutput = 0.8, 

                                   distal_max_percentoutput_per_second = 12.0,
                                   distal_calibration_time = 0.5, 

                                   distal_min_angle = -45, 
                                   distal_max_angle = 90, 

                                   distal_gear_ratio = 680.0 / 3.0 * 1.2, 

                                   second_motor_tolerance = 4, 


                                   
                                   min_difference = 20, 

                                   first_arm_length = 32 - 4, // subtract 4 from length of arm to get pivot distance 
                                   second_arm_length = 15 + 7.751984, // add 7.751984 to length of arm to get pivot distance

                                   min_x = -2, 
                                   min_y = -42,
                                   max_x = 61.368, 
                                   max_y = 20, 

                                   middle_x = 45, // above this, we go at a slower rate
                                   middle_y = 0, 
                                   speed_when_arm_extended = 0.33; // TODO Change this later once the arm actually works
    }
    
    public static final class IntakeConstants {
        public static final double claw_current_limit = 25, 
                                   outtake_time = 0.5;
    }

    public static final class SwerveConstants {
        
        public static final double swerve_max_pid_rotation = 0.8, 
                                   swerve_calibration_time = 0.5, // seconds

                                   swerve_turn_sensitivity = 0.75, 
                                   swerve_high_turn_sensitivity = 0.35, 
                                   swerve_min_translation = 0.05, 
                                   swerve_min_rotation = 0.02, 

                                   yaw_tolerance = 2, 
                                   swerve_slow_factor = 0.2, 
                                   tffsmbtcfigfdbnft = 2, // turn factor for slow motion because the current factor is good for distance but not for turning
                                   vision_min_error = 15; // degree diff between imu yaw and vision yaw
    }

    public static final class Buttons {

        /* D-pad */

        public static final int dpad_up = 0, 
                                dpad_right = 90, 
                                dpad_down = 180, 
                                dpad_left = 270;

        /* Which is which */

        public static final int joystick_usb_port = 2, 
                                driver_usb_port = 1, 
                                operator_usb_port = 0;

        /* Driver Buttons */

        public static final int translationAxis = XboxController.Axis.kLeftY.value, 
                                strafeAxis = XboxController.Axis.kLeftX.value,

                                rotationAxis = XboxController.Axis.kRightX.value,

                                directionXAxis = XboxController.Axis.kRightX.value, 
                                directionYAxis = XboxController.Axis.kRightY.value, 
                                turnRightAxis = XboxController.Axis.kRightTrigger.value, 
                                turnLeftAxis = XboxController.Axis.kLeftTrigger.value, 
                                
                                slowAxis = XboxController.Axis.kRightTrigger.value, 
                                slowButtonOne = XboxController.Button.kRightBumper.value, 
                                slowButtonTwo = XboxController.Button.kLeftBumper.value, 

                                robotCentricButton = XboxController.Button.kLeftBumper.value,

                                zeroGyroButton = XboxController.Button.kY.value, 
                                makeXButton = XboxController.Button.kX.value, 
                                stopUsingEncodersButtonOne = XboxController.Button.kA.value, 
                                stopUsingEncodersButtonTwo = XboxController.Button.kB.value,
                                terminateCommandsDriverButton = XboxController.Button.kBack.value,

                                joystickTranslationAxis = 1, // backward is positive
                                joystickStrafeAxis = 0, // right is positive
                                joystickRotationAxis = 5, // right is positive
                                joystickSlowButton = 1,
                                joystickZeroGyroButton = 3, 
                                joystickMakeXButton = 2, 
                                joystickTerminateCommandsButton = 4, // if we push the slider forward it cancels commands
                                joystickResetArmEncodersButton = 12, // make this like some really obscure button that would literally only be pressed on purpose
                                joystickAutoConeOuttakeButton = 11;
                                // .getPOV() is the same for joystick; T.Flight Hotas One

        /* Operator Buttons */

        public static final int motorOneAxis = XboxController.Axis.kLeftY.value, 
                                motorTwoAxis = XboxController.Axis.kRightY.value, 

                                topGoalButton = XboxController.Button.kY.value, 
                                midGoalButton = XboxController.Button.kB.value, 
                                lowGoalButton = XboxController.Button.kA.value, 

                                intakeUpperValue = dpad_up, 
                                intakeLowerValue = dpad_down, 

                                intakeButton = XboxController.Button.kRightBumper.value, 
                                outtakeButton = XboxController.Button.kLeftBumper.value, 

                                idleButton = XboxController.Button.kStart.value, 

                                stopArmFromMovingButtonOne = XboxController.Axis.kRightTrigger.value,
                                stopArmFromMovingButtonTwo = XboxController.Axis.kLeftTrigger.value,
                                terminateCommandsOperatorButton = XboxController.Button.kBack.value;
    }

    public static final class GeneralConstants {

        public static final double joystick_deadzone = 0.2,
                                   axis_exponent = 1.3;

        public static final double signedPower(double axis_value) {
            axis_value = MathUtil.applyDeadband(axis_value, joystick_deadzone);
            if (axis_value == 0) return 0;
            if (axis_value < 0) return 0 - Math.pow(0 - axis_value, axis_exponent);
            return Math.pow(axis_value, axis_exponent);
        }

        public static final double getCorrection(double error, double min_error, double max_error, double exponent, double max_power) {
            if (Math.abs(error) < min_error) return 0;
            double multiplier = max_power;
            if (error < 0) {
                error = 0 - error;
                multiplier = -max_power;
            }
            if (error > max_error) error = max_error;
            error /= max_error;
            return Math.pow(error, exponent) * multiplier;
        }

        /** Normalizes angle to between -180 and 180 */
        public static double normalizeAngle(double degrees) {
            if (degrees < 0) return ((degrees - 180) % 360 + 180);
            return ((degrees + 180) % 360 - 180);
        }

        public static double getPECorrection(double error, double p, double e, double max_power) {
            double correction = error * p;
            if (Math.abs(correction) < max_power) {
                correction *= Math.pow(Math.abs(error * p) / max_power, e - 1);
            }
            return correction;
        }
    }

    public static final class TuningConstants {
        public static final double CANCoder0_zero = 268.15, 
                                   CANCoder1_zero = 314.03, 
                                   CANCoder2_zero = 317.98, 
                                   CANCoder3_zero = 184.3, 
                                   encoder_one_zero = -92.73, 
                                   encoder_two_zero = -305.55;
        
        /* The angles we want to go to - NOT the x and y */
        public static final double[] topPositionAngles = {-11.6, 41.0}, 
                                     midPositionAngles = {-62.6, 65.0}, 
                                     lowPositionAngles = {-64, 0}, 

                                     intakeUpperAngles = {-106.5, 40.6}, 
                                     intakeLowerAngles = {-83.8, -32.3}, 

                                     idlePositionAngles = {-115, 42}; // TODO Make X cancel arm pid for if its swinging crazy

        public static final boolean fancy_drive = true, 
                                    genericHID_drive = true,
                                    testmode = true;
        
        /* Swerve Drive Constants */

        public static final double drive_motor_p = 0.05,
                                   drive_static_voltage = 0.32, 
                                   drive_equilibrium_voltage = 1.51, 
                                   drive_acceleration_voltage = 0.27, // SYSID values: KS, KV, KA; they are automatically divided by 12 later
                                   max_linear_speed = 10, // feet per second; theoretical max is 13.5
                                   max_angular_speed = 180; // degrees per second; theoretical max is theoretical maximum is max_linear_speed * 42.7402910527 which is roughly 500

        /* Auto Constants */

        public static final double max_linear_speed_autonomous = 8,
                                   ramp_up_time_linear = 2.5, // in seconds to reach max 
                                   max_angular_speed_autonomous = 180, 
                                   ramp_up_time_angular = 1.5,
                                   translation_p_controller = 2, 
                                   angle_p_controller = 4;
        
        /* Arm Constants */

        public static final double p1 = 0.045, 
                                   e1 = 1.1, 
                                   d1 = 0, 
                                   i1 = 0, 
                                   p2 = 0.075, 
                                   e2 = 1.4, 
                                   d2 = 0, 
                                   i2 = 0, 
                                   pS = 0.1, 
                                   eS = 1.5, 
                                   dS = 0, 
                                   iS = 0,
                                   mftfmitdaiwtmdoftoabtc = 0.65, // mutiplier for the first motor if the distal arm is within the minimum distance of the first one allowed by the code
                                   mftfmitdaiwtmdoftoabtcaiicgd = 0.4, // and it is currently going down
                                   pT = 0.4,
                                   dT = 0.2;
    }
}