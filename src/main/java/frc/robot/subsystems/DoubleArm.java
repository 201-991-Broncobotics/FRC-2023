package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DoubleArmConstants.*;

public class DoubleArm extends SubsystemBase {

    private CANSparkMax first_motor, first_motor_follower, second_motor;
    private DutyCycleEncoder first_encoder, second_encoder;

    private double[] target_positions = new double[2];

    private double time, time_one_last, time_two_last;

    private int first_motor_mode = 0, second_motor_mode = 0;

    public DoubleArm() { // Initialize the motors, encoders, and target positions
        first_motor = new CANSparkMax(first_motor_ID, MotorType.kBrushless); // NEO Motors are brushless
        first_motor_follower = new CANSparkMax(first_motor_follower_ID, MotorType.kBrushless);
        second_motor = new CANSparkMax(second_motor_ID, MotorType.kBrushless);

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

        target_positions = getCurrentArmAngles();

        Timer.delay(1.0);

        time = Timer.getFPGATimestamp();
        time_one_last = 0;
        time_two_last = 0;
    }

    public void powerArm(double firstPower, double secondPower) { // power the arms manually

        double[] current_angles = getCurrentArmAngles();
        double delta_time = Timer.getFPGATimestamp() - time; // in seconds
        time = Timer.getFPGATimestamp();

        if (first_motor_mode * firstPower > 0) {
            firstPower = 0;
        } else if (firstPower != 0) {
            first_motor_mode = 0;
        }
        
        if (second_motor_mode * secondPower > 0) { // technically I could simplify this to firstPower * first_motor_mode > 0
            secondPower = 0;
        } else if (secondPower != 0) {
            second_motor_mode = 0;
        }

        if (current_angles[0] < min_first_angle) { // don't power it down until we manually power it up again
            firstPower = Math.max(correction_ratio * first_motor_max_power, firstPower);
            first_motor_mode = -1; // don't allow manual power to be down
        } else if (current_angles[0] > max_first_angle) {
            firstPower = Math.min(-correction_ratio * first_motor_max_power, firstPower);
            first_motor_mode = 1;
        }

        if (current_angles[1] < min_second_angle) {
            secondPower = Math.max(correction_ratio * second_motor_max_power, secondPower);
            second_motor_mode = -1;
        } else if (current_angles[1] > Math.min(max_second_angle, current_angles[0] + 180 - min_difference)) {
            secondPower = Math.min(-correction_ratio * second_motor_max_power, secondPower);
            second_motor_mode = 1;
        }

        if (!checkTargetAngles(current_angles)) { // out of bounds

            System.out.println("current position is out of bounds");

            if (getPositionFromAngles(current_angles)[1] > max_y) {
                // it means that we should only power down

                first_motor_mode = 1;
                second_motor_mode = 1;

                if (current_angles[1] > 0) {
                    firstPower = Math.min(firstPower, 0);
                    secondPower = Math.min(secondPower, -correction_ratio * second_motor_max_power);
                } else {
                    firstPower = Math.min(firstPower, -correction_ratio * first_motor_max_power);
                    secondPower = Math.min(secondPower, 0);
                }
            } else if (getPositionFromAngles(current_angles)[0] < min_x) {
                // only power first motor up, second toward 0

                first_motor_mode = -1;

                firstPower = Math.max(firstPower, 0);
                
                if (current_angles[1] > 0) {
                    second_motor_mode = 1;
                    secondPower = Math.min(secondPower, 0);
                } else {
                    second_motor_mode = -1;
                    secondPower = Math.max(secondPower, 0);
                }
            } else { // must be less than min_y because not possible to be greater than max_x
                // power second one a bit to correct
                second_motor_mode = -1;
                secondPower = Math.max(secondPower, correction_ratio * second_motor_max_power);
            }
        }

        if (firstPower != 0) {
            target_positions[0] = current_angles[0];
            time_one_last = time;
        } else if (time - time_one_last < whiplash_time_one) {
            target_positions[0] = current_angles[0];
        } else {
            firstPower = pidPower(
                target_positions[0] - current_angles[0], 
                first_motor_max_power, 
                first_motor_min_error, 
                first_motor_max_error
            );
        }

        if (secondPower != 0) {
            target_positions[1] = current_angles[1];
            time_two_last = time;
        } else if (time - time_two_last < whiplash_time_two) {
            target_positions[1] = current_angles[1];
        } else {
            secondPower = pidPower(
                target_positions[1] - current_angles[1], 
                second_motor_max_power, 
                second_motor_min_error, 
                second_motor_max_error
            );
        }

        firstPower = Math.max(first_motor.get() - first_motor_max_acceleration * delta_time, Math.min(first_motor.get() + first_motor_max_acceleration * delta_time, firstPower));
        secondPower = Math.max(second_motor.get() - second_motor_max_acceleration * delta_time, Math.min(second_motor.get() + second_motor_max_acceleration * delta_time, secondPower));

        first_motor.set(firstPower);
        second_motor.set(secondPower);
    }

    public void bangbang(boolean first_arm) {

        first_motor_mode = 0;
        second_motor_mode = 0;

        double[] current_angles = getCurrentArmAngles();
        double delta_time = Timer.getFPGATimestamp() - time; // in seconds
        time = Timer.getFPGATimestamp();

        double firstPower;
        double secondPower;

        if (first_arm) {

            firstPower = target_positions[0] > current_angles[0] ? first_motor_bangbang_power : -first_motor_bangbang_power;

            secondPower = pidPower(
                target_positions[1] - current_angles[1], 
                second_motor_max_power, 
                second_motor_min_error, 
                second_motor_max_error
            );

        } else {

            firstPower = pidPower(
                target_positions[0] - current_angles[0], 
                first_motor_max_power, 
                first_motor_min_error, 
                first_motor_max_error
            );

            secondPower = target_positions[1] > current_angles[1] ? second_motor_bangbang_power : -second_motor_bangbang_power_two;

            if (Math.abs(target_positions[1] - current_angles[1]) < second_motor_constant_min_error) {
                secondPower *= nonconstant_multiplier_one; // if we're within 15 degrees don't go to the full bangbang power
            }
        }
        
        firstPower = Math.max(first_motor.get() - first_motor_max_acceleration * delta_time, Math.min(first_motor.get() + first_motor_max_acceleration * delta_time, firstPower));
        secondPower = Math.max(second_motor.get() - second_motor_max_acceleration * delta_time, Math.min(second_motor.get() + second_motor_max_acceleration * delta_time, secondPower));

        first_motor.set(firstPower);
        second_motor.set(secondPower);
    }

    public void powerProximalMaxDistal() { // powers the proximal while keeping the maximal distal at all times
        
        first_motor_mode = 0;
        second_motor_mode = 0;

        double[] current_angles = getCurrentArmAngles();
        double delta_time = Timer.getFPGATimestamp() - time; // in seconds
        time = Timer.getFPGATimestamp();

        double firstPower = target_positions[0] > current_angles[0] ? first_motor_bangbang_power : -first_motor_bangbang_power;

        if (Math.abs(target_positions[0] - current_angles[0]) < first_motor_constant_min_error) {
            firstPower *= nonconstant_multiplier_one; // if we're within 15 degrees don't go to the full bangbang power
        }
        
        target_positions[1] = Math.min(Math.min(90, max_second_angle), current_angles[0] + 180 - min_difference);
        
        double secondPower = target_positions[1] > current_angles[1] ? second_motor_bangbang_power_two : -second_motor_bangbang_power_two_down;

        if (Math.abs(target_positions[1] - current_angles[1]) < second_motor_constant_min_error) {
            secondPower *= nonconstant_multiplier_two; // if we're within 15 degrees don't go to the full bangbang power
        }

        firstPower = Math.max(first_motor.get() - first_motor_max_acceleration * delta_time, Math.min(first_motor.get() + first_motor_max_acceleration * delta_time, firstPower));
        secondPower = Math.max(second_motor.get() - second_motor_max_acceleration * delta_time, Math.min(second_motor.get() + second_motor_max_acceleration * delta_time, secondPower));

        first_motor.set(firstPower);
        second_motor.set(secondPower);
    }

    public void brake() {
        first_motor.set(0);
        second_motor.set(0);
    }

    public void resetWhipControl() {
        time_one_last = -999;
        time_two_last = -999;
    }

    public void resetPID() {
        target_positions[0] = getCurrentArmAngles()[0];
        target_positions[1] = getCurrentArmAngles()[1];
        brake();
    }

    public void setTargetAngles(double[] target_angles) {
        // Sets the 2 target angles if they both fit the requirements

        target_angles[0] = Math.max(min_first_angle, Math.min(max_first_angle, target_angles[0]));
        target_angles[1] = Math.max(min_second_angle, Math.min(Math.min(max_second_angle, target_angles[0] + 180 - min_difference), target_angles[1]));

        target_positions = target_angles;
        if (!checkTargetAngles(target_angles)) {
            SmartDashboard.putNumberArray("Position failed", target_angles);
        }
    }

    public void setTargetPosition(double[] target) {
        setTargetAngles(getAnglesFromTarget(target));
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

    public static double[] getAnglesFromTarget(double[] target) {

        double x = target[0];
        double y = target[1];

        x = Math.max(min_x, Math.min(max_x, x));
        y = Math.max(min_y, Math.min(max_y, y));

        if (Math.abs(x) < 0.01) x = 0.01; // shouldn't matter because min_x > 0

        double radius = Math.sqrt(x * x + y * y);
        if (radius < clipping_one * (first_arm_length - second_arm_length)) {
            x *= clipping_one * (first_arm_length - second_arm_length) / radius;
            y *= clipping_one * (first_arm_length - second_arm_length) / radius;
            radius = clipping_one * (first_arm_length - second_arm_length);
        } else if (radius > clipping_two * (first_arm_length + second_arm_length)) {
            x *= clipping_two * (first_arm_length + second_arm_length) / radius;
            y *= clipping_two * (first_arm_length + second_arm_length) / radius;
            radius = clipping_two * (first_arm_length + second_arm_length);
        } // clip radius to range

        double angle = Math.atan(y / x) * 180.0 / Math.PI; 
        if (x < 0) {
            if (angle > 0) angle -= 180;
            else angle += 180; // again, this should NOT happen
        }

        double first_angle = Math.acos((radius * radius + first_arm_length * first_arm_length - second_arm_length * second_arm_length) / (2.0 * first_arm_length * radius)) * 180.0 / Math.PI;
        double second_angle = Math.acos((radius * radius + second_arm_length * second_arm_length - first_arm_length * first_arm_length) / (2.0 * second_arm_length * radius)) * 180.0 / Math.PI;

        return new double[] {
            angle + (angle < switching_angle ? 0 - first_angle : first_angle),
            angle + (angle < switching_angle ? second_angle : 0 - second_angle)
        };
    }

    public double[] getCurrentArmAngles() {
        return new double[] {
            first_encoder.getDistance() - first_encoder_zero, 
            first_encoder.getDistance() - first_encoder_zero + second_encoder.getDistance() - second_encoder_zero
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
            target_positions[0], 
            target_positions[1]
        };
    }

    public double getTotalError() {
        double[] target_xy = getPositionFromAngles(target_positions);
        return Math.sqrt(
            (getCurrentXY()[0] - target_xy[0]) * (getCurrentXY()[0] - target_xy[0]) + 
            (getCurrentXY()[1] - target_xy[1]) * (getCurrentXY()[1] - target_xy[1])
        );
    }

    public double pidPower(double error, double maxPower, double minDegreesOff, double maxDegreesOff) {
        int multiplier = 1;
        if (error < 0) {
            error = 0 - error;
            multiplier = -1;
        }

        if (error < minDegreesOff) return 0;

        error /= maxDegreesOff;
        if (error > 1) error = 1;

        return maxPower * multiplier * Math.pow(error, k_exponent);
    }


    @Override
    public void periodic() { // Put data to smart dashboard
        SmartDashboard.putNumber("Encoder 1 Raw", first_encoder.getDistance());
        SmartDashboard.putNumber("Encoder 2 Raw", second_encoder.getDistance());

        SmartDashboard.putNumber("Current Angle 1", getCurrentArmAngles()[0]);
        SmartDashboard.putNumber("Current Angle 2", getCurrentArmAngles()[1]);

        SmartDashboard.putNumber("Current x", getCurrentXY()[0]);
        SmartDashboard.putNumber("Current y", getCurrentXY()[1]);

        SmartDashboard.putNumber("Target 1st Angle", target_positions[0]);
        SmartDashboard.putNumber("Target 2nd Angle", target_positions[1]);

        SmartDashboard.putNumber("Target x", getPositionFromAngles(target_positions)[0]);
        SmartDashboard.putNumber("Target y", getPositionFromAngles(target_positions)[1]);

        SmartDashboard.putNumber("Error", getTotalError());

        SmartDashboard.putNumber("Motor One Current", first_motor.getOutputCurrent());
        SmartDashboard.putNumber("Motor One Follower Current", first_motor_follower.getOutputCurrent());
        SmartDashboard.putNumber("Motor Two Current", second_motor.getOutputCurrent());

        if ((getCurrentXY()[0] < middle_x) && (getCurrentXY()[1] < middle_y)) {
            frc.robot.Variables.speed_factor = 1;
        } else {
            frc.robot.Variables.speed_factor = speed_when_arm_extended;
        }

        /* if (target_positions[0] > 20) {
            brake();
            System.out.println("Bad x and y" + getPositionFromAngles(target_positions)[0] + " " + getPositionFromAngles(target_positions)[1]);
            System.out.println("Bad angles" + target_positions[0] + " " + target_positions[1]);
            Timer.delay(0.5);
            throw new IllegalArgumentException();
        } */
        // target_positions[0] = Math.max(-120, Math.min(20, target_positions[0]));
        // target_positions[1] = Math.max(-80, Math.min(80, target_positions[1]));
        // target_xy = getPositionFromAngles(target_positions[0], target_positions[1]);
    }
}