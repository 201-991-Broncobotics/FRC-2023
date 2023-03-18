package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class PIDCalculator {
    private final double kP, kD, kI, kE, maxPower;

    private double previous_time, previous_error, integral, correction, targetPosition;
    private boolean justReset;

    public PIDCalculator(double kP, double kD, double kI, double maxPower, double startingPosition) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kE = 1;
        this.maxPower = maxPower;
        reset(startingPosition);
    }
    
    public PIDCalculator(double kP, double kD, double kI, double kE, double maxPower, double startingPosition) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kE = kE;
        this.maxPower = maxPower;
        reset(startingPosition);
    }

    public void reset(double targetPosition) { // resets integral
        this.targetPosition = targetPosition;
        integral = 0;
        previous_time = Timer.getFPGATimestamp();
        justReset = true;
    }

    public void forceReset(double targetPosition) { // doesn't reset integral
        this.targetPosition = targetPosition;
        previous_time = Timer.getFPGATimestamp();
        justReset = true;
    }

    public double update(double currentPosition) {

        double error = targetPosition - currentPosition;
        double delta_time = Timer.getFPGATimestamp() - previous_time;
        double delta_error = error - previous_error;

        previous_time = Timer.getFPGATimestamp();
        previous_error = error;

        double p, i, d;

        p = error * kP;
        if (Math.abs(p) < maxPower) {
            p *= Math.pow(Math.abs(error * kP) / maxPower, kE - 1);
        }

        if (justReset) { // only return p
            justReset = false;
            previous_error = error;
            return p;
        }
        if (delta_time != 0) {
            d = delta_error / delta_time * kD;
        } else {
            d = 0;
        }
        integral += error * delta_time;
        i = integral * kI;
        correction = Math.max(-1, Math.min(1, p + i + d)) * maxPower;
        return correction;
    }

    public double getCorrection() {
        return correction;
    }

    public double getTarget() {
        return targetPosition;
    }
}
