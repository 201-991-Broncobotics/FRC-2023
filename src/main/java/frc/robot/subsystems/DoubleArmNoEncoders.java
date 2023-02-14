package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DoubleArmConstants.*;

public class DoubleArmNoEncoders extends SubsystemBase {

    private CANSparkMax first_motor, first_motor_follower, second_motor;

    public DoubleArmNoEncoders() { // Initialize the motors, encoders, and target positions
        first_motor = new CANSparkMax(first_motor_ID, MotorType.kBrushless); // NEO Motors are brushless
        first_motor_follower = new CANSparkMax(first_motor_follower_ID, MotorType.kBrushless);
        second_motor = new CANSparkMax(second_motor_ID, MotorType.kBrushless);

        first_motor.restoreFactoryDefaults();
        first_motor.setIdleMode(IdleMode.kBrake);
        first_motor.setInverted(invert_first_motor);

        first_motor_follower.restoreFactoryDefaults();
        first_motor_follower.setIdleMode(IdleMode.kBrake);
        first_motor_follower.setInverted(invert_first_motor);
        
        second_motor.restoreFactoryDefaults();
        second_motor.setIdleMode(IdleMode.kBrake);
        second_motor.setInverted(invert_first_motor);
        
        // we want it so powering the motors rotates the arms counterclockwise

        /* Hopefully these are fine with the defaults :)
        first_motor.setOpenLoopRampRate(first_motor_acceleration_time); // maximum time for 0 to full throttle
        first_motor.setSmartCurrentLimit(first_motor_max_current); // current limit
        first_motor.enableVoltageCompensation(first_motor_voltage_compensation);
        
        first_motor_follower.setOpenLoopRampRate(first_motor_acceleration_time);
        first_motor_follower.setSmartCurrentLimit(first_motor_max_current); // current limit
        first_motor_follower.enableVoltageCompensation(first_motor_voltage_compensation);
        
        second_motor.setOpenLoopRampRate(second_motor_acceleration_time);
        second_motor.setSmartCurrentLimit(second_motor_max_current); // current limit
        second_motor.enableVoltageCompensation(second_motor_voltage_compensation); */

        first_motor_follower.follow(first_motor, invert_first_motor_follower); // follows it in the same direction
                    // we may have to change this part, I'm not sure

        Timer.delay(1.0);
    }

    public void rawPowerArm(double firstPower, double secondPower) {
        first_motor.set(firstPower);
        second_motor.set(secondPower);
    }

    @Override
    public void periodic() { // Put data to smart dashboard
        SmartDashboard.putNumber("Arm 1 Current", first_motor.getOutputCurrent());

        SmartDashboard.putNumber("Default Ramp Rate", first_motor.getOpenLoopRampRate());
        SmartDashboard.putNumber("First Motor Current", first_motor.getOutputCurrent());
        SmartDashboard.putNumber("Default Voltage Compensation", first_motor.getVoltageCompensationNominalVoltage());
    }
}