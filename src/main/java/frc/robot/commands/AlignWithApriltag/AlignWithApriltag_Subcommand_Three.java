package frc.robot.commands.AlignWithApriltag;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_Three extends CommandBase {
    
    // Strafe left or right until we can see the april tag

    private Swerve swerve;
    private BooleanSupplier exitSup;

    public AlignWithApriltag_Subcommand_Three(Swerve swerve, BooleanSupplier exitSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.exitSup = exitSup;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
