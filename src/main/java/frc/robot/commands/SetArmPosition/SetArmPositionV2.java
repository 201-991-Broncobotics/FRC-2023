package frc.robot.commands.setArmPosition;

import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmPositionV2 extends SequentialCommandGroup { // in theory this should be better in every single way
    
    public SetArmPositionV2(DoubleArm doubleArm, double[] position) {
        addRequirements(doubleArm);
        addCommands(
            new SetMaximalDistalVersionDos(doubleArm), 
            new SetProximalPositionV2(doubleArm, position[0]), 
            new SetDistalPosition(doubleArm, position[1]) // if this works, I might as well just make this a true PID
        );
    }

}

class SetMaximalDistalVersionDos extends CommandBase {

    private DoubleArm doubleArm;
    private double distalPosition;
    private boolean greater;

    public SetMaximalDistalVersionDos(DoubleArm doubleArm) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);
    }

    @Override
    public void initialize() {
        doubleArm.resetPID();
        
        distalPosition = Math.min(Math.min(90, max_second_angle), doubleArm.getCurrentArmAngles()[0] + 180 - min_difference);

        doubleArm.setTargetAngles(new double[] {doubleArm.getCurrentArmAngles()[0], distalPosition});
        greater = doubleArm.getCurrentArmAngles()[1] < distalPosition;
    }

    @Override
    public void execute() {
        doubleArm.bangbang(false);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            doubleArm.resetPID();
        }
    }
    
    @Override
    public boolean isFinished() {
        return (doubleArm.getTotalError() < tolerance) || ((doubleArm.getCurrentArmAngles()[1] > distalPosition) == greater);
    }
}

class SetProximalPositionV2 extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double proximalPosition;
    private boolean greater;

    public SetProximalPositionV2(DoubleArm doubleArm, double proximalPosition) {
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
        doubleArm.powerProximalMaxDistal();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            doubleArm.resetPID();
        }
    }
    
    @Override
    public boolean isFinished() {
        return (doubleArm.getTotalError() < tolerance) || ((doubleArm.getCurrentArmAngles()[0] > proximalPosition) == greater);
    }
    
}