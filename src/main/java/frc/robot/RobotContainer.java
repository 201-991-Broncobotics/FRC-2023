package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.*;
import frc.robot.commands.autoDriveCommands.DriveToNearestCone;
import frc.robot.commands.defaultCommands.*;
import frc.robot.commands.setArmPosition.*;
import frc.robot.commands.setClawState.*;
import frc.robot.commands.utilCommands.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.Buttons.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.TuningConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(driver_usb_port);
    private final XboxController operator = new XboxController(operator_usb_port);
    private final GenericHID driver_joystick = new GenericHID(joystick_usb_port);

    /* Both Controllers */
    private final Trigger terminateCommands = new JoystickButton(driver, terminateCommandsDriverButton).or(new JoystickButton(operator, terminateCommandsOperatorButton)).or(new JoystickButton(driver_joystick, joystickTerminateCommandsButton));

    /* Driver Buttons */
    private final Trigger zeroGyro = new JoystickButton(driver, zeroGyroButton).or(new JoystickButton(driver_joystick, joystickZeroGyroButton));
    private final Trigger makeX = new JoystickButton(driver, makeXButton).or(new JoystickButton(driver_joystick, joystickMakeXButton));

    private final Trigger robotCentric = new JoystickButton(driver, robotCentricButton);

    private final Trigger autoConeOuttake = new JoystickButton(driver_joystick, joystickAutoConeOuttakeButton);

    private final Trigger resetArmEncoders = new Trigger(() -> (
        (driver.getRawButton(stopUsingEncodersButtonOne)) &&
        (driver.getRawButton(stopUsingEncodersButtonTwo))
    )).or(new Trigger(() -> driver_joystick.getRawButton(joystickResetArmEncodersButton)));

    /* Operator Buttons */
    private final Trigger topGoal = new JoystickButton(operator, topGoalButton);
    private final Trigger midGoal = new JoystickButton(operator, midGoalButton);
    private final Trigger lowGoal = new JoystickButton(operator, lowGoalButton);

    private final Trigger idle = new JoystickButton(operator, idleButton);

    private final Trigger intake = new JoystickButton(operator, intakeButton);
    private final Trigger outtake = new JoystickButton(operator, outtakeButton);

    private final Trigger stopArmCommands = new Trigger(() -> (
        (operator.getRawAxis(stopArmFromMovingButtonOne) > joystick_deadzone) || 
        (operator.getRawAxis(stopArmFromMovingButtonTwo) > joystick_deadzone)
    ));

    private final Trigger intakeUpper = new Trigger(() -> (operator.getPOV() == intakeUpperValue));
    private final Trigger intakeLower = new Trigger(() -> (operator.getPOV() == intakeLowerValue));

    /* Custom Triggers */

    private final Trigger goToIdlePosition = new Trigger(() -> (frc.robot.Variables.go_to_startposition));

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final DoubleArm doubleArm = new DoubleArm();
    private final Claw claw = new Claw(); // testing purposes, if there's an error then comment this back out

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        if (genericHID_drive) {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver_joystick.getRawAxis(joystickTranslationAxis), 
                    () -> -driver_joystick.getRawAxis(joystickStrafeAxis), 
                    () -> -driver_joystick.getRawAxis(joystickRotationAxis), 
                    () -> false, 
                    () -> -driver_joystick.getPOV(), 
                    () -> {
                        if (driver_joystick.getRawButton(joystickSlowButton)) {
                            return swerve_slow_factor;
                        } else {
                            return 1.0;
                        }
                    }
                     // what we multiply translation speed by; rotation speed is NOT affected
                )
            );
        } else if (fancy_drive) {
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

        doubleArm.setDefaultCommand(
            new TeleopDoubleArm(
                doubleArm, 
                () -> -operator.getRawAxis(motorOneAxis),
                () -> -operator.getRawAxis(motorTwoAxis)
            )
        );

        claw.setDefaultCommand(
            new TeleOpClaw(claw)
        );

        // Cache autonomous commands
        Autonomous.cacheCommandGroups(s_Swerve, doubleArm, claw);

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

        /* Both Controllers */
        terminateCommands.toggleOnTrue(new TerminateCommands(claw, doubleArm, s_Swerve));

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        makeX.onTrue(new Brake(s_Swerve));

        resetArmEncoders.toggleOnTrue(new InstantCommand(() -> doubleArm.stopUsingEncoders()));
        autoConeOuttake.toggleOnTrue(new DriveToNearestCone(s_Swerve, doubleArm, claw));
        
        /* Operator Buttons */
        topGoal.toggleOnTrue(new SetArmPosition(doubleArm, topPositionAngles));
        midGoal.toggleOnTrue(new SetArmPosition(doubleArm, midPositionAngles));
        lowGoal.toggleOnTrue(new SetArmPosition(doubleArm, lowPositionAngles));
        idle.toggleOnTrue(new SetArmPosition(doubleArm, idlePositionAngles));

        intakeUpper.toggleOnTrue(new SetArmPosition(doubleArm, intakeUpperAngles));
        intakeLower.toggleOnTrue(new SetArmPosition(doubleArm, intakeLowerAngles));

        stopArmCommands.onTrue(new TerminateArmCommands(doubleArm));

        intake.toggleOnTrue(new Intake(claw));
        outtake.toggleOnTrue(new Outtake(claw, s_Swerve));

        /* Custom Triggers */
        goToIdlePosition.toggleOnTrue(new SetArmPositionAfterIntake(claw, doubleArm));
    }

    public void teleopInit() {
        doubleArm.teleOpInit();
        doubleArm.resetPID();
        s_Swerve.changeHeading(0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Autonomous.getAutonomousCommand(claw, doubleArm, s_Swerve);
    }

    public Swerve getSwerve() {
        return s_Swerve;
    }

    public DoubleArm getDoubleArm() {
        return doubleArm;
    }

    public Claw getClaw() {
        return claw;
    }
}
