package frc.robot.commands.AlignWithApriltag;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_Five extends CommandBase {
    
    // Strafe until we are in line with april tag, then finally brake 

    private Swerve swerve;
    private BooleanSupplier exitSup;

    public AlignWithApriltag_Subcommand_Five(Swerve swerve, BooleanSupplier exitSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.exitSup = exitSup;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
