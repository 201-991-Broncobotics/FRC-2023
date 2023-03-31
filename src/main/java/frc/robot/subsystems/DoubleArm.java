package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDTalon;

import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.TuningConstants.*;

import java.util.function.BooleanSupplier;

public class DoubleArm extends SubsystemBase {

    private static ArmMode mode = ArmMode.CUBE; 

    private final PIDTalon proximal, distal; 
    private final DutyCycleEncoder first_encoder, second_encoder;

    private boolean useEncoders = true;

    public DoubleArm() { // Initialize the motors, encoders, and target positions

        first_encoder = new DutyCycleEncoder(first_encoder_channel);
        second_encoder = new DutyCycleEncoder(second_encoder_channel);

        first_encoder.setDistancePerRotation(360.0 * (invert_first_encoder ? -1 : 1)); // no gear ratio
        second_encoder.setDistancePerRotation(360.0 * (invert_second_encoder ? -1 : 1));

        proximal = new PIDTalon(
            proximal_id, proximal_max_continuous_current, proximal_max_peak_current, proximal_peak_current_time, proximal_open_ramp_rate, proximal_closed_ramp_rate, proximal_brake, invert_proximal, getInitialArmAngles()[0], proximal_min_angle, proximal_max_angle, proximal_max_percentoutput, proximal_max_percentoutput_per_second, proximal_gear_ratio, invert_proximal_encoder, proximal_calibration_time, p1, d1, i1, e1, proximal_follower_id
        );

        
        distal = new PIDTalon(
            distal_id, distal_max_continuous_current, distal_max_peak_current, distal_peak_current_time, distal_open_ramp_rate, distal_closed_ramp_rate, distal_brake, invert_distal, getInitialArmAngles()[1], distal_min_angle, distal_max_angle, distal_max_percentoutput, distal_max_percentoutput_per_second, distal_gear_ratio, invert_distal_encoder, distal_calibration_time, p2, d2, i2, e2
        );

        // we want it so powering the motors rotates the arms counterclockwise
        
        Timer.delay(1.0); // invert correctly
    }

    public void teleOpInit() {
        useEncoders = true;
        proximal.resetTarget();
        distal.resetTarget();
    }

    public void powerArm(double firstPower, double secondPower) { // power the arms manually
        
        if (useEncoders) {
            double[] current_angles = getCurrentArmAngles();
            if (current_angles[1] > current_angles[0] + 180 - min_difference) {
                distal.setTarget(current_angles[0] + 180 - min_difference);
                secondPower = Math.min(secondPower, 0);
            } // all of zay others are gewd

            if (!checkTargetAngles(current_angles)) { // out of bounds

                if (getPositionFromAngles(current_angles)[1] > max_y) {

                    firstPower = Math.min(firstPower, 0);

                    secondPower = Math.min(secondPower, 0);

                    double delta_y = max_y - first_arm_length * Math.sin(current_angles[0] * Math.PI / 180.0);
                    if (Math.abs(delta_y) < second_arm_length - 0.5) {
                        distal.setTarget(Math.asin(delta_y / second_arm_length) * 180.0 / Math.PI);
                    } else {
                        distal.setTarget(Math.min(90, distal_max_angle));
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
                        distal.setTarget(Math.asin(delta_y / second_arm_length) * 180.0 / Math.PI);
                    } else {
                        distal.setTarget(Math.max(-90, distal_min_angle));
                    }
                }
            }
        }

        proximal.power(firstPower);
        distal.power(secondPower);
    }

    public void pidPowerArm() {
        proximal.pidPower();
        distal.pidPower();
    }

    public void pidPowerKeepMaxDistal() {

        double mftfmbtdaiwtmdoftoabtc = 1;
        if (Math.min(Math.min(distal_max_angle, 90), getCurrentArmAngles()[0] + 180 - min_difference) < Math.min(distal_max_angle, 90)) {
            if (proximal.getTarget() > getCurrentArmAngles()[0]) {
                mftfmbtdaiwtmdoftoabtc = mftfmitdaiwtmdoftoabtc;
            } else {
                mftfmbtdaiwtmdoftoabtc = mftfmitdaiwtmdoftoabtcaiicgd;
            }
            distal.setTarget(Math.min(proximal.getTarget() + 180 - min_difference, getCurrentArmAngles()[0] + 180 - min_difference));
        } else {
            distal.setTarget(Math.min(distal_max_angle, 90));
        }
        proximal.pidPower(mftfmbtdaiwtmdoftoabtc);
        distal.pidPower();
    }

    public void brake() {
        proximal.brake();
        distal.brake();
    }

    public void resetToAbsolute() {
        proximal.resetSensorPosition(getInitialArmAngles()[0]);
        distal.resetSensorPosition(getInitialArmAngles()[1]);
    }

    public void resetPID() {
        proximal.brake();
        distal.brake();
        proximal.resetTarget();
        distal.resetTarget();
    }

    public void stopUsingEncoders() {
        useEncoders = false;
        proximal.disableLimiting();
        distal.disableLimiting();
    }

    public boolean getUsingEncoders() {
        return useEncoders;
    }

    public void setTargetAngles(double[] angles) {
        proximal.setTarget(angles[0]);
        distal.setTarget(angles[1]);
    }

    public double[] getInitialArmAngles() {
        return new double[] {
            normalizeAngle(first_encoder.getDistance() - first_encoder_zero), 
            normalizeAngle(first_encoder.getDistance() - first_encoder_zero + second_encoder.getDistance() - second_encoder_zero)
        };
    }

    public double[] getCurrentArmAngles() {
        return new double[] {
            proximal.getEncoderPosition(), 
            distal.getEncoderPosition()
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
            proximal.getTarget(), 
            distal.getTarget()
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

        // SmartDashboard.putNumber("Current x", getCurrentXY()[0]);
        // SmartDashboard.putNumber("Current y", getCurrentXY()[1]);

        SmartDashboard.putNumber("Target 1st Angle", getTargetArmAngles()[0]); // getInitialArmAngles
        SmartDashboard.putNumber("Target 2nd Angle", getTargetArmAngles()[1]);

        // SmartDashboard.putNumber("Target x", getPositionFromAngles(getTargetArmAngles())[0]);
        // SmartDashboard.putNumber("Target y", getPositionFromAngles(getTargetArmAngles())[1]);

        // SmartDashboard.putNumber("Error", getTotalError());
        
        SmartDashboard.putNumber("Motor One Current", proximal.getCurrent());
        SmartDashboard.putNumber("Motor Two Current", distal.getCurrent());

        if (((getCurrentXY()[0] < middle_x) && (getCurrentXY()[1] < middle_y)) || (!useEncoders)) {
            frc.robot.Variables.speed_factor = 1;
        } else {
            frc.robot.Variables.speed_factor = speed_when_arm_extended;
        }

        if (getCurrentArmAngles()[1] < 0 && distal.getCurrent() > second_motor_stop_current && frc.robot.Variables.thor && Timer.getFPGATimestamp() - frc.robot.Variables.ats > 2 && Timer.getFPGATimestamp() - frc.robot.Variables.ats < 5) {
            throw new IllegalArgumentException("Please don't destroy the robot");
        }
    }

    public static void setToCubeMode() {
        SmartDashboard.putString("ArmMode", "CUBE");
        mode = ArmMode.CUBE;
    }

    public static void setToConeMode() {
        SmartDashboard.putString("ArmMode", "CONE");
        mode = ArmMode.CONE;
    }

    public static BooleanSupplier isConeMode = () -> mode == ArmMode.CONE;
}

enum ArmMode {
    CUBE, 
    CONE
}