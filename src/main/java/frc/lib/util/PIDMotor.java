package frc.lib.util;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;

public class PIDMotor {
    private final CANSparkMax motor;
    private final DoubleSupplier positionSup;
    private final PIDCalculator pidCalculator;
    private final double calibrationTime, maxAcceleration;
    private double minPosition, maxPosition;

    private double previousTime, time, lmtPosition;

    public PIDMotor(CANSparkMax motor, DoubleSupplier positionSup, double calibrationTime, double minPosition, double maxPosition, double maxPower, double maxAcceleration, double kP, double kD, double kI, double kE) {
        this.motor = motor;
        this.positionSup = positionSup;
        this.calibrationTime = calibrationTime;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.maxAcceleration = maxAcceleration;
        pidCalculator = new PIDCalculator(kP, kD, kI, kE, maxPower, positionSup.getAsDouble());
        time = Timer.getFPGATimestamp();
        previousTime = -1000;
    }

    public void power(double power) {
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();

        double currentPosition = positionSup.getAsDouble();

        if (currentPosition < minPosition || lmtPosition <= minPosition) {
            previousTime = -1000;
            power = Math.max(0, power);
            motor.set(power);
            pidCalculator.reset(minPosition);
            lmtPosition = minPosition;
        } else if (currentPosition > maxPosition || lmtPosition >= maxPosition) {
            previousTime = -1000;
            power = Math.min(0, power);
            motor.set(power);
            pidCalculator.reset(maxPosition);
            lmtPosition = maxPosition;
        }

        if (power != 0) {
            pidCalculator.reset(currentPosition);
            lmtPosition = currentPosition;
            previousTime = time;
        } else if (time - previousTime < calibrationTime) {
            pidCalculator.reset(currentPosition);
        } else {
            power = pidCalculator.update(currentPosition);
        }
        
        power = Math.max(motor.get() - maxAcceleration * deltaTime, Math.min(motor.get() + maxAcceleration * deltaTime, power));

        motor.set(power);
    }

    public void resetTarget() {
        previousTime = -1000;
        pidCalculator.reset(positionSup.getAsDouble());
        motor.set(0);
        lmtPosition = positionSup.getAsDouble();
    }

    public void setTarget(double targetPosition) {
        previousTime = -1000;
        pidCalculator.reset(targetPosition);
        motor.set(0);
        lmtPosition = targetPosition;
    }

    public void pidPower() {
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        lmtPosition = 0; // between min and max
        double power = pidCalculator.update(positionSup.getAsDouble());
        power = Math.max(motor.get() - maxAcceleration * deltaTime, Math.min(motor.get() + maxAcceleration * deltaTime, power));

        motor.set(power);
    }

    public void pidPower(double multiplier) {
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        lmtPosition = 0; // between min and max
        double power = pidCalculator.update(positionSup.getAsDouble()) * multiplier;
        power = Math.max(motor.get() - maxAcceleration * deltaTime, Math.min(motor.get() + maxAcceleration * deltaTime, power));

        motor.set(power);
    }

    public void brake() {
        pidCalculator.reset(positionSup.getAsDouble());
        previousTime = Timer.getFPGATimestamp();
        motor.set(0);
    }

    public double getTarget() {
        return pidCalculator.getTarget();
    }

    public void disableLimiting() {
        minPosition = Double.NEGATIVE_INFINITY;
        maxPosition = Double.POSITIVE_INFINITY;
    }
}
