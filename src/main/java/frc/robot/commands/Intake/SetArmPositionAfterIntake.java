package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.TuningConstants.*;

public class SetArmPositionAfterIntake extends SequentialCommandGroup {

    public SetArmPositionAfterIntake(Claw claw, DoubleArm doubleArm) {
        addRequirements(claw, doubleArm);
        addCommands(
            new InstantCommand(() -> { claw.stop(); frc.robot.Variables.go_to_startposition = false; })// , 
            // new ConditionalCommand(new SetArmPosition(doubleArm, idlePositionAngles), new InstantCommand(), () -> doubleArm.getUsingEncoders())
        );
    }
}
