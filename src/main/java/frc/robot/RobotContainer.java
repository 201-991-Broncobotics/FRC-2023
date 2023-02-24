package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.AlignWithApriltag.AlignWithApriltagOld;
import frc.robot.commands.AutoBalance.AutoBalance;
import frc.robot.commands.Intake.Intake;
import frc.robot.commands.Outtake.Outtake;
import frc.robot.commands.SetArmPosition.SetArmPosition;
import frc.robot.subsystems.*;

import static frc.robot.Constants.Buttons.*;
import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.autos.autoTrajectories.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(1);
    private final XboxController operator = new XboxController(0);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, zeroGyroButton);
    private final JoystickButton robotCentric = new JoystickButton(driver, robotCentricButton);
    private final JoystickButton tagAligner = new JoystickButton(driver, tagAlignerButton);
    private final JoystickButton brake = new JoystickButton(driver, brakeButton);
    private final JoystickButton autoBalance = new JoystickButton(driver, autoBalanceButton);
    private final JoystickButton terminateCommandsDriver = new JoystickButton(driver, terminateCommandsDriverButton);

    /* Operator Buttons */
    private final JoystickButton topGoal = new JoystickButton(operator, topGoalButton);
    private final JoystickButton midGoal = new JoystickButton(operator, midGoalButton);
    private final JoystickButton lowGoal = new JoystickButton(operator, lowGoalButton);

    private final JoystickButton idle = new JoystickButton(operator, idleButton);
    private final JoystickButton startPos = new JoystickButton(operator, startPosButton);

    private final JoystickButton intake = new JoystickButton(operator, intakeButton);
    private final JoystickButton outtake = new JoystickButton(operator, outtakeButton);
    
    private final JoystickButton terminateCommandsOperator = new JoystickButton(operator, terminateCommandsOperatorButton);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final DoubleArm doubleArm = new DoubleArm();
    private final Claw claw = new Claw(); // testing purposes, if there's an error then comment this back out

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        if (fancy_drive) {
            s_Swerve.setDefaultCommand(
                new TeleopSwerveAbsoluteDirecting(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(directionXAxis), 
                    () -> -driver.getRawAxis(directionYAxis), 
                    () -> -driver.getPOV(), 
                    () -> driver.getRawAxis(turnLeftAxis) - driver.getRawAxis(turnRightAxis), 
                    () -> (driver.getRawButton(slowButtonOne) || driver.getRawButton(slowButtonTwo)))
            );
        } else {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    () -> robotCentric.getAsBoolean(), 
                    () -> -driver.getPOV(), 
                    () -> 1 - 0.75 * driver.getRawAxis(slowAxis) // what we multiply translation speed by; rotation speed is NOT affected
                )
            );
        }

        if (manual_control) {
            doubleArm.setDefaultCommand(
                new TeleopDoubleArmManualControl(
                    doubleArm, 
                    () -> -operator.getRawAxis(motorOneAxis),
                    () -> -operator.getRawAxis(motorTwoAxis)
                )
            );
        } else {
            doubleArm.setDefaultCommand(
                new TeleopDoubleArmCartesianControl(
                    doubleArm, 
                    () -> operator.getRawAxis(horizAxis),
                    () -> -operator.getRawAxis(vertAxis)
                )
            );
        }

        // No default command for Claw

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        tagAligner.toggleOnFalse(new AlignWithApriltagOld(s_Swerve, () -> driver.getRawButton(tagAlignerExitButton)));
                    // toggle on false because otherwise it automatically stops it
        
        brake.toggleOnFalse(new Brake(s_Swerve));

        autoBalance.toggleOnFalse(new AutoBalance(s_Swerve, doubleArm, () -> driver.getRawButton(autoBalanceExitButton)));

        terminateCommandsDriver.toggleOnTrue(new TerminateCommands(claw, doubleArm, s_Swerve));
        
        /* Operator Buttons */
        topGoal.toggleOnTrue(new SetArmPosition(doubleArm, topPosition));
        midGoal.toggleOnTrue(new SetArmPosition(doubleArm, midPosition));
        lowGoal.toggleOnTrue(new SetArmPosition(doubleArm, lowPosition));
        idle.toggleOnTrue(new SetArmPosition(doubleArm, idlePosition));
        startPos.toggleOnTrue(new SetArmPosition(doubleArm, startPosition));

        intake.toggleOnFalse(new Intake(claw, doubleArm, () -> operator.getRawButton(intakeButton)));
        outtake.toggleOnFalse(new Outtake(claw, doubleArm));
        
        terminateCommandsOperator.toggleOnTrue(new TerminateCommands(claw, doubleArm, s_Swerve));
    }

    public void teleopInit() {
        doubleArm.resetPID();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);
        return new emulatorAuto(s_Swerve, doubleArm, testTrajectory);
    }
}
