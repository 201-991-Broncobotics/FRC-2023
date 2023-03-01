package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

import java.util.function.BooleanSupplier;

public class SetDistalPosition extends CommandBase { // small arm

    private DoubleArm doubleArm;
    private double distalPosition;
    private BooleanSupplier stopSup;

    public SetDistalPosition(DoubleArm doubleArm, double distalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        stopSup = () -> false;
        this.distalPosition = distalPosition;
    }

    public SetDistalPosition(DoubleArm doubleArm, double distalPosition, BooleanSupplier stopSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);
        
        this.stopSup = stopSup;
        this.distalPosition = distalPosition;
    }

    @Override
    public void initialize() {
        if (!stopSup.getAsBoolean())
            doubleArm.setTargetAngles(new double[] {doubleArm.getTargetArmAngles()[0], distalPosition});
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
