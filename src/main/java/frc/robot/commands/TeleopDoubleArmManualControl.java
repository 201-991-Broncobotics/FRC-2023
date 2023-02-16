package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class TeleopDoubleArmManualControl extends CommandBase {

    private DoubleArm doubleArm;

    private DoubleSupplier motorOneSup;
    private DoubleSupplier motorTwoSup;

    private double previous_first_power;
    private double previous_second_power;

    public TeleopDoubleArmManualControl(DoubleArm doubleArm, DoubleSupplier motorOneSup, DoubleSupplier motorTwoSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.motorOneSup = motorOneSup;
        this.motorTwoSup = motorTwoSup; // sets up the double suppliers
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorOneVal = MathUtil.applyDeadband(motorOneSup.getAsDouble(), joystick_deadzone);
        double motorTwoVal = MathUtil.applyDeadband(motorTwoSup.getAsDouble(), joystick_deadzone);

        double current_first_power = motorOneVal * raw_arm_sensitivity;
        if (current_first_power == 0 && Math.abs(previous_first_power) > tolerance_power) {
            current_first_power = previous_first_power * axeponent;
        }
        previous_first_power = current_first_power;

        double current_second_power = motorTwoVal * raw_arm_sensitivity_two;
        if (current_second_power == 0 && Math.abs(previous_second_power) > tolerance_power) {
            current_second_power = previous_second_power * axeponent;
        }
        previous_second_power = current_second_power;

        // Move Arm
        doubleArm.rawPowerArm(
            current_first_power, // power????? but it's weird
            current_second_power
        );
    }
}