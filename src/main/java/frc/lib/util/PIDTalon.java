package frc.lib.util;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;

public class PIDTalon {

    private final TalonFX motor;
    private final PIDCalculator pidCalculator;

    private final DoubleSupplier positionSup;
    private final double calibrationTime, maxPercentOutputPerSecond;
    private double minPosition, maxPosition, previousTime, time, lmtPosition, prevPower;

    public PIDTalon(
        int CanID, double continuousCurrentLimit, double peakCurrentLimit, double peakCurrentTime, double openRampRate, double closedRampRate, 
        boolean brake, boolean inverted, double startingAngle, double minPosition, double maxPosition, double maxPercentOutput, double maxPercentOutputPerSecond, 
        double gear_ratio, boolean invertEncoder, double calibrationTime, double kP, double kD, double kI, double kE, int ... followerIDs
    ) {

        TalonFXConfiguration Config = new TalonFXConfiguration();

        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 
            continuousCurrentLimit, 
            peakCurrentLimit, 
            peakCurrentTime
        );

        NeutralMode neutralMode = brake ? NeutralMode.Brake : NeutralMode.Coast;

        Config.supplyCurrLimit = driveSupplyLimit;
        Config.openloopRamp = openRampRate;
        Config.closedloopRamp = closedRampRate;

        this.motor = new TalonFX(CanID);

        motor.configFactoryDefault();
        motor.configAllSettings(Config);
        motor.setInverted(inverted);
        Timer.delay(1.0);
        motor.setNeutralMode(neutralMode);
        motor.setSelectedSensorPosition(startingAngle * 256.0 / 45.0 * (invertEncoder ? -1 : 1) * gear_ratio); // 2048 per revolution and its in degrees
        // motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);

        for (int i : followerIDs) {
            TalonFX tempMotor = new TalonFX(i);
            tempMotor.configFactoryDefault();
            tempMotor.configAllSettings(Config);
            tempMotor.setInverted(inverted);
            tempMotor.setNeutralMode(neutralMode);
            // tempMotor.enableVoltageCompSaturation(12); // idk what's happening here
            tempMotor.enableVoltageCompensation(true);
            tempMotor.follow(motor);
        }

        Timer.delay(1.0);

        this.minPosition = minPosition;
        this.maxPosition = maxPosition;

        this.calibrationTime = calibrationTime;
        this.maxPercentOutputPerSecond = maxPercentOutputPerSecond;

        positionSup = () -> motor.getSelectedSensorPosition() * 45.0 / 256.0 * (invertEncoder ? -1 : 1) / gear_ratio;

        pidCalculator = new PIDCalculator(kP, kD, kI, kE, maxPercentOutput, startingAngle);
        time = Timer.getFPGATimestamp();
        previousTime = -1000;
    }

    public void power(double power) { // power = between 0 and 1
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();

        double currentPosition = positionSup.getAsDouble();
        
        if (currentPosition < minPosition || lmtPosition <= minPosition) {
            previousTime = -1000;
            power = Math.max(0, power);
            prevPower = power;
            motor.set(ControlMode.PercentOutput, 0);
            pidCalculator.reset(minPosition);
            lmtPosition = minPosition;
        } else if (currentPosition > maxPosition || lmtPosition >= maxPosition) {
            previousTime = -1000;
            power = Math.min(0, power);
            prevPower = power;
            motor.set(ControlMode.PercentOutput, 0);
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

        power = Math.max(prevPower - maxPercentOutputPerSecond * deltaTime, Math.min(prevPower + maxPercentOutputPerSecond * deltaTime, power));
        prevPower = power;

        motor.set(ControlMode.PercentOutput, power);
    }

    public void resetTarget() {
        previousTime = -1000;
        pidCalculator.reset(positionSup.getAsDouble());
        prevPower = 0;
        motor.set(ControlMode.PercentOutput, 0);
        lmtPosition = positionSup.getAsDouble();
    }

    public void setTarget(double targetPosition) {
        previousTime = -1000;
        pidCalculator.reset(targetPosition);
        prevPower = 0;
        motor.set(ControlMode.PercentOutput, 0);
        lmtPosition = targetPosition;
    }

    public void pidPower() {
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        lmtPosition = 0; // between min and max
        double power = pidCalculator.update(positionSup.getAsDouble());
        power = Math.max(prevPower - maxPercentOutputPerSecond * deltaTime, Math.min(prevPower + maxPercentOutputPerSecond * deltaTime, power));
        prevPower = power;

        motor.set(ControlMode.PercentOutput, power);
    }

    public void pidPower(double multiplier) {
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        lmtPosition = 0; // between min and max
        double power = pidCalculator.update(positionSup.getAsDouble()) * multiplier;
        power = Math.max(prevPower - maxPercentOutputPerSecond * deltaTime, Math.min(prevPower + maxPercentOutputPerSecond * deltaTime, power));
        prevPower = power;

        motor.set(ControlMode.PercentOutput, power);
    }

    public void brake() {
        pidCalculator.reset(positionSup.getAsDouble());
        previousTime = Timer.getFPGATimestamp();
        prevPower = 0;
        motor.set(ControlMode.PercentOutput, 0);
    }

    public double getTarget() {
        return pidCalculator.getTarget();
    }

    public void disableLimiting() {
        minPosition = Double.NEGATIVE_INFINITY;
        maxPosition = Double.POSITIVE_INFINITY;
    }

    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    public double getEncoderPosition() {
        return positionSup.getAsDouble();
    }
}
