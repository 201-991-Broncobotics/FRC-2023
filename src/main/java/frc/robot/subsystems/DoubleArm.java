package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDMotor;

import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.TuningConstants.*;

public class DoubleArm extends SubsystemBase {

    private CANSparkMax first_motor, first_motor_follower, second_motor;
    private DutyCycleEncoder first_encoder, second_encoder;
    private PIDMotor first, second;
    private boolean useEncoders = true;

    public DoubleArm() { // Initialize the motors, encoders, and target positions
        first_motor = new CANSparkMax(first_motor_ID, MotorType.kBrushless); // NEO Motors are brushless
        first_motor_follower = new CANSparkMax(first_motor_follower_ID, MotorType.kBrushless);
        second_motor = new CANSparkMax(second_motor_ID, MotorType.kBrushed);

        first_motor.restoreFactoryDefaults();
        first_motor.setInverted(invert_first_motor);

        first_motor_follower.restoreFactoryDefaults();
        first_motor_follower.setInverted(invert_first_motor);
        
        second_motor.restoreFactoryDefaults();
        second_motor.setInverted(invert_second_motor);

        if (first_motor_brake) {
            first_motor.setIdleMode(IdleMode.kBrake);
            first_motor_follower.setIdleMode(IdleMode.kBrake);
        } else {
            first_motor.setIdleMode(IdleMode.kCoast);
            first_motor_follower.setIdleMode(IdleMode.kCoast);
        }

        if (second_motor_brake) {
            second_motor.setIdleMode(IdleMode.kBrake);
        } else {
            second_motor.setIdleMode(IdleMode.kCoast);
        }
        
        // we want it so powering the motors rotates the arms counterclockwise
        
        first_motor.setSmartCurrentLimit(first_motor_max_current); // current limit
        first_motor_follower.setSmartCurrentLimit(first_motor_max_current);
        second_motor.setSmartCurrentLimit(second_motor_max_current);

        /* Hopefully these are fine with the defaults
        first_motor.setOpenLoopRampRate(first_motor_acceleration_time); // maximum time for 0 to full throttle
        first_motor.enableVoltageCompensation(first_motor_voltage_compensation);
        
        first_motor_follower.setOpenLoopRampRate(first_motor_acceleration_time);
        first_motor_follower.enableVoltageCompensation(first_motor_voltage_compensation);
        
        second_motor.setOpenLoopRampRate(second_motor_acceleration_time);
        second_motor.enableVoltageCompensation(second_motor_voltage_compensation); */

        first_motor_follower.follow(first_motor, invert_first_motor_follower); // follows it in the same direction
                    // we may have to change this part, I'm not sure

        first_encoder = new DutyCycleEncoder(first_encoder_channel);
        second_encoder = new DutyCycleEncoder(second_encoder_channel);

        first_encoder.setDistancePerRotation(360.0 * (invert_first_encoder ? -1 : 1)); // no gear ratio
        second_encoder.setDistancePerRotation(360.0 * (invert_second_encoder ? -1 : 1));
        
        // we want it so the encoder increases when the arm goes counterclockwise - may have to adjust them

        Timer.delay(1.0); // invert correctly

        first = new PIDMotor(first_motor, () -> getCurrentArmAngles()[0], whiplash_time_one, first_motor_min_angle, first_motor_max_angle, first_motor_max_power, first_motor_max_acceleration, p1, d1, i1);
        second = new PIDMotor(second_motor, () -> getCurrentArmAngles()[1], whiplash_time_two, second_motor_min_angle, second_motor_max_angle, second_motor_max_power, second_motor_max_acceleration, p2, d2, i2);
    }

    public void teleOpInit() {
        useEncoders = true;
        first.resetTarget();
        second.resetTarget();
    }

    public void powerArm(double firstPower, double secondPower) { // power the arms manually

        if (useEncoders) {
            double[] current_angles = getCurrentArmAngles();
            if (current_angles[1] > current_angles[0] + 180 - min_difference) {
                second.setTarget(current_angles[0] + 180 - min_difference);
                secondPower = Math.min(secondPower, 0);
            } // all of zay others are gewd

            if (!checkTargetAngles(current_angles)) { // out of bounds

                if (getPositionFromAngles(current_angles)[1] > max_y) {

                    firstPower = Math.min(firstPower, 0);

                    secondPower = Math.min(secondPower, 0);

                    double delta_y = max_y - first_arm_length * Math.sin(current_angles[0] * Math.PI / 180.0);
                    if (Math.abs(delta_y) < second_arm_length - 0.5) {
                        second.setTarget(Math.asin(delta_y / second_arm_length) * 180.0 / Math.PI);
                    } else {
                        second.setTarget(Math.min(90, second_motor_max_angle));
                    }

                } else if (getPositionFromAngles(current_angles)[0] < min_x) {

                    firstPower = Math.max(firstPower, 0);
                    
                    if (current_angles[1] > 0) {
                        secondPower = Math.min(secondPower, 0);
                    } else {
                        secondPower = Math.max(secondPower, 0);
                    }

                } else { // must be less than min_y because not possible to be greater than max_x
                    
                    secondPower = Math.max(secondPower, 0);
                    
                    double delta_y = min_y - first_arm_length * Math.sin(current_angles[0] * Math.PI / 180.0);
                    if (Math.abs(delta_y) < second_arm_length - 0.5) {
                        second.setTarget(Math.asin(delta_y / second_arm_length) * 180.0 / Math.PI);
                    } else {
                        second.setTarget(Math.max(-90, second_motor_min_angle));
                    }
                }
            }
        }

        first.power(firstPower);
        second.power(secondPower);
    }

    public void pidPowerArm() {
        first.pidPower();
        second.pidPower();
    }

    public void pidPowerKeepMaxDistal() {
        first.pidPower();

        second.setTarget(Math.min(Math.min(second_motor_max_angle, 90), getCurrentArmAngles()[0] + 180 - min_difference));
        second.pidPower();
    }

    public void brake() {
        first.brake();
        second.brake();
    }

    public void resetPID() {
        first.brake();
        second.brake();
    }

    public void stopUsingEncoders() {
        useEncoders = false;
        first.disableLimiting();
        second.disableLimiting();
    }

    public boolean getUsingEncoders() {
        return useEncoders;
    }

    public void setTargetAngles(double[] angles) {
        first.setTarget(angles[0]);
        second.setTarget(angles[1]);
    }

    public double[] getCurrentArmAngles() {
        // TODO: use arm encoders
        // first_motor.getAbsoluteEncoder() maybe
        return new double[] {
            normalizeAngle(first_encoder.getDistance() - first_encoder_zero), 
            normalizeAngle(first_encoder.getDistance() - first_encoder_zero + second_encoder.getDistance() - second_encoder_zero)
        };
    }
    
    public double[] getCurrentXY() {
        return new double[] {
            first_arm_length * Math.cos(getCurrentArmAngles()[0] * Math.PI / 180.0) + 
            second_arm_length * Math.cos(getCurrentArmAngles()[1] * Math.PI / 180.0), 
            first_arm_length * Math.sin(getCurrentArmAngles()[0] * Math.PI / 180.0) + 
            second_arm_length * Math.sin(getCurrentArmAngles()[1] * Math.PI / 180.0)
        };
    }

    public double[] getTargetArmAngles() {
        return new double[] {
            first.getTarget(), 
            second.getTarget()
        };
    }

    public double getTotalError() {
        double[] target_xy = getPositionFromAngles(getTargetArmAngles());
        return Math.sqrt(
            (getCurrentXY()[0] - target_xy[0]) * (getCurrentXY()[0] - target_xy[0]) + 
            (getCurrentXY()[1] - target_xy[1]) * (getCurrentXY()[1] - target_xy[1])
        );
    }

    public static boolean checkTargetAngles(double[] angles) {
        double x = getPositionFromAngles(angles)[0];
        double y = getPositionFromAngles(angles)[1];
        if (x < min_x || x > max_x) return false;
        if (y < min_y || y > max_y) return false;
        return true;
    }
    
    public static double[] getPositionFromAngles(double[] angles) {
        return new double[] {
            first_arm_length * Math.cos(angles[0] * Math.PI / 180.0) + 
            second_arm_length * Math.cos(angles[1] * Math.PI / 180.0), 
            first_arm_length * Math.sin(angles[0] * Math.PI / 180.0) + 
            second_arm_length * Math.sin(angles[1] * Math.PI / 180.0)
        };
    }

    @Override
    public void periodic() { // Put data to smart dashboard
        SmartDashboard.putNumber("Encoder 1 Raw", first_encoder.getDistance());
        SmartDashboard.putNumber("Encoder 2 Raw", second_encoder.getDistance());

        SmartDashboard.putNumber("Current Angle 1", getCurrentArmAngles()[0]);
        SmartDashboard.putNumber("Current Angle 2", getCurrentArmAngles()[1]);

        SmartDashboard.putNumber("Current x", getCurrentXY()[0]);
        SmartDashboard.putNumber("Current y", getCurrentXY()[1]);

        SmartDashboard.putNumber("Target 1st Angle", getTargetArmAngles()[0]);
        SmartDashboard.putNumber("Target 2nd Angle", getTargetArmAngles()[1]);

        SmartDashboard.putNumber("Target x", getPositionFromAngles(getTargetArmAngles())[0]);
        SmartDashboard.putNumber("Target y", getPositionFromAngles(getTargetArmAngles())[1]);

        SmartDashboard.putNumber("Error", getTotalError());
        
        SmartDashboard.putNumber("Motor One Current", first_motor.getOutputCurrent());
        SmartDashboard.putNumber("Motor One Follower Current", first_motor_follower.getOutputCurrent());
        SmartDashboard.putNumber("Motor Two Current", second_motor.getOutputCurrent());

        if (((getCurrentXY()[0] < middle_x) && (getCurrentXY()[1] < middle_y)) || (!useEncoders)) {
            frc.robot.Variables.speed_factor = 1;
        } else {
            frc.robot.Variables.speed_factor = speed_when_arm_extended;
        }
    }
}