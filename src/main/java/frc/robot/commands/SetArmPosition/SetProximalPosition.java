package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

public class SetProximalPosition extends CommandBase {
    
    private DoubleArm doubleArm;
    private double proximalPosition;

    public SetProximalPosition(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.proximalPosition = proximalPosition;
    }

    @Override
    public void initialize() {
        doubleArm.setTargetAngles(new double[] {proximalPosition, doubleArm.getTargetArmAngles()[1]});
    }

    @Override
    public void execute() {
        doubleArm.pidPowerKeepMaxDistal();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            doubleArm.resetPID();
        }
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(doubleArm.getCurrentArmAngles()[0] - proximalPosition) < first_motor_tolerance;
    }
    
}