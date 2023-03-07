package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

public class SetProximalPosition extends CommandBase {
    
    private DoubleArm doubleArm;
    private double proximalPosition;
    private boolean greater;

    public SetProximalPosition(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.proximalPosition = proximalPosition;
    }

    @Override
    public void initialize() {
        doubleArm.setTargetAngles(new double[] {proximalPosition, doubleArm.getTargetArmAngles()[1]});
        greater = doubleArm.getCurrentArmAngles()[0] < proximalPosition;
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
        return (doubleArm.getCurrentArmAngles()[0] > proximalPosition) == greater;
    }
    
}