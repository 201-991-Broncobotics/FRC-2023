package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

import static frc.robot.Constants.Buttons.*;

public class TeleopDoubleArmCartesianControl extends CommandBase {

    private DoubleArm doubleArm;

    private DoubleSupplier horizSup;
    private DoubleSupplier vertSup;

    private double time;

    public TeleopDoubleArmCartesianControl(DoubleArm doubleArm, DoubleSupplier horizSup, DoubleSupplier vertSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.horizSup = horizSup;
        this.vertSup = vertSup; // sets up the double suppliers
        time = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double horizVal = Math.pow(MathUtil.applyDeadband(horizSup.getAsDouble(), joystick_deadzone), axis_exponent);
        double vertVal = Math.pow(MathUtil.applyDeadband(vertSup.getAsDouble(), joystick_deadzone), axis_exponent);

        double delta_time = System.currentTimeMillis() / 1000.0 - time;
        time = System.currentTimeMillis() / 1000.0;
        // Move Arm
        doubleArm.moveArm(
            horizVal * arm_sensitivity * delta_time, 
            vertVal * arm_sensitivity * delta_time
        );
    }

}