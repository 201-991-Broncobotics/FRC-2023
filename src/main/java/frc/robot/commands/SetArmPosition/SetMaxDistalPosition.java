package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

import java.util.function.BooleanSupplier;

public class SetMaxDistalPosition extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double distalPosition;
    private BooleanSupplier stopSup;

    public SetMaxDistalPosition(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        stopSup = () -> false;
        distalPosition = proximalPosition + 180 - min_difference;
    }

    public SetMaxDistalPosition(DoubleArm doubleArm, double proximalPosition, BooleanSupplier stopSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.stopSup = stopSup;
        distalPosition = proximalPosition + 180 - min_difference;
    }

    @Override
    public void initialize() { // we only want to run if our target proximal is above the current
        if (!stopSup.getAsBoolean()) {
            double target_distal = Math.max(
                doubleArm.getCurrentArmAngles()[0] + 180 - min_difference, 
                distalPosition
            ); // maximal possible distal position

            doubleArm.setTargetAngles(new double[] {doubleArm.getCurrentArmAngles()[0], target_distal});
        }
    }

    @Override
    public void execute() {
        doubleArm.PIDPowerArm();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // code for if it ended by interruption
        } else {
            // code for if it ended by reaching the target position
            // if we manually move it it will say it's finished
            doubleArm.brake();
        }
    }
    
    @Override
    public boolean isFinished() {
        if (stopSup.getAsBoolean()) {
            doubleArm.resetPID();
            return true;
        }
        return doubleArm.getTotalError() < tolerance;
    }
}
