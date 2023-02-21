package frc.robot.commands.AlignWithApriltag;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_One extends CommandBase {

    // Calculate heading of april tag, using current heading

    private Swerve swerve;
    private BooleanSupplier exitSup;

    public AlignWithApriltag_Subcommand_One(Swerve swerve, BooleanSupplier exitSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.exitSup = exitSup;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
