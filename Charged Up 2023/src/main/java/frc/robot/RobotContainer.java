// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.autonomousCommands.AutoBalanceTesting;
import frc.robot.commands.autonomousCommands.FinalAutonCubeOnHigh.*;
import frc.robot.commands.autonomousCommands.drive.*;
import frc.robot.commands.routineCommands.*;
import frc.robot.commands.teleopCommands.armExtension.*;
import frc.robot.commands.teleopCommands.armshoulder.*;
import frc.robot.commands.teleopCommands.drive.*;
import frc.robot.commands.teleopCommands.intake.*;
//subsystem imports
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
        private static final ArmExtension m_ArmExtension = new ArmExtension();
        private static final ArmShoulder m_ArmShoulder = new ArmShoulder(m_ArmExtension);
        private static final IntakeSubsystem m_intake = new IntakeSubsystem();
        private final CameraSubsystem cameraSubsystem = new CameraSubsystem();

        // Auton commands:
        private final Basic m_PlaceCubeOnHigh = new Basic(m_ArmShoulder, m_ArmExtension, m_intake);
        private final AutoBalanceBackwards m_AutoBalanceBackwards = new AutoBalanceBackwards(m_driveSubsystem);
        private final AutoBalanceTesting m_AutoBalanceTesting = new AutoBalanceTesting(m_driveSubsystem,
                m_AutoBalanceBackwards);
        private final RightStartStickout m_RightStartStickout = new RightStartStickout(m_ArmShoulder, m_ArmExtension, m_intake, m_driveSubsystem);
        private final LeftStartStickout m_LeftStartStickout = new LeftStartStickout(m_ArmShoulder, m_ArmExtension, m_intake, m_driveSubsystem);
        private final MiddleStart m_MiddleStart = new MiddleStart(m_ArmShoulder, m_ArmExtension, m_intake,
                m_driveSubsystem);
        private final RightStart m_RightStart = new RightStart(m_ArmShoulder, m_ArmExtension, m_intake, m_driveSubsystem);
        private final LeftStart m_LeftStart = new LeftStart(m_ArmShoulder, m_ArmExtension, m_intake, m_driveSubsystem);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Replace with CommandPS4Controller or CommandJoystick if needed

    // THE FIRST CONTOLLER PLUGGED IN CONTROLS THE DRIVETRAIN, THE SECOND CONTROLLER
    // PLUGGED IN CONTROLS THE ARM/INTAKE
    private final XboxController m_driverStick = new XboxController(IOConstants.kDriverStick);
    //private final XboxController m_operatorStick = new XboxController(IOConstants.kOperatorStick);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        m_driveSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> m_driveSubsystem.slowSpeedDriveArcade(m_driverStick.getRawAxis(Constants.IOConstants.kLY),
                                m_driverStick.getRawAxis(Constants.IOConstants.kRX)),
                        m_driveSubsystem));

        // AUTON COMMANDS
        m_chooser.setDefaultOption("Left Stickout Start", m_LeftStartStickout);
        m_chooser.addOption("Right Stickout Start", m_RightStartStickout);
        m_chooser.addOption("Left Start", m_LeftStart);
        m_chooser.addOption("Right Start", m_RightStart);
        m_chooser.addOption("AutoBalance Test", m_AutoBalanceTesting);
        m_chooser.addOption("Place Cube on High", m_PlaceCubeOnHigh);
        m_chooser.addOption("Middle Start", m_MiddleStart);
        SmartDashboard.putData(m_chooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        // EXTENSION BINDINGS 
        new POVButton(m_driverStick, 0).onTrue(new ExtendLevel0(m_ArmExtension));
        new POVButton(m_driverStick, 270).onTrue(new ExtendLevel1(m_ArmExtension));
        new POVButton(m_driverStick, 180).onTrue(new ExtendLevel2(m_ArmExtension));
        new POVButton(m_driverStick, 90).onTrue(new ExtendLevel3(m_ArmExtension));

        // EXTRA BINDINGs
        new JoystickButton(m_driverStick, IOConstants.kSTART).onTrue(new PickupCone(m_ArmShoulder, m_ArmExtension, m_intake));
        new JoystickButton(m_driverStick, IOConstants.kBACK).onTrue(new RotateLevel3(m_ArmShoulder));


        // COMPOUND ARM MOVEMENT BINDINGS
        new JoystickButton(m_driverStick, IOConstants.kA).onTrue(new ArmLevel0(m_ArmShoulder, m_ArmExtension));

        new JoystickButton(m_driverStick, IOConstants.kB).onTrue(new ConditionalCommand(
                new ArmLevel1In(m_ArmShoulder, m_ArmExtension),
                new ArmLevel1Out(m_ArmShoulder, m_ArmExtension),
                m_ArmShoulder::isArmAboveLevel1));

        new JoystickButton(m_driverStick, IOConstants.kY).onTrue(new ConditionalCommand(
                new ArmLevel2Parallel(m_ArmShoulder, m_ArmExtension),
                new ArmLevel2Sequential(m_ArmShoulder, m_ArmExtension),
                m_ArmShoulder::isArmAboveLevel1));

        new JoystickButton(m_driverStick, IOConstants.kX).onTrue(new ConditionalCommand(
                new ArmLevel3Parallel(m_ArmShoulder, m_ArmExtension),
                new ArmLevel3Sequential(m_ArmShoulder, m_ArmExtension),
                m_ArmShoulder::isArmAboveLevel1));

        // INTAKE BINDINGS

        new JoystickButton(m_driverStick, IOConstants.kLB).whileTrue(new suckCone(m_intake));
        new JoystickButton(m_driverStick, IOConstants.kRB).whileTrue(new suckCube(m_intake));
        new Trigger(() -> m_driverStick.getRawAxis(IOConstants.kRT) > 0.5)
                .whileTrue(new shootCube(m_intake));

        new Trigger(() -> m_driverStick.getRawAxis(IOConstants.kLT) > 0.5)
                .whileTrue(new shootCone(m_intake));

        // SLOW DRIVE BINDINGS
        // new JoystickButton(m_driverStick, IOConstants.kA).whileTrue(new SlowDriveForward(m_driveSubsystem));
        // new JoystickButton(m_driverStick, IOConstants.kY).whileTrue(new SlowDriveBackward(m_driveSubsystem));


        // // SLOW TURN BINDINGS
        // new JoystickButton(m_driverStick, IOConstants.kLB).whileTrue(new SlowTurnRight(m_driveSubsystem));
        // new JoystickButton(m_driverStick, IOConstants.kRB).whileTrue(new SlowTurnLeft(m_driveSubsystem));

        // // TURN 90 DEGREES BINDINGS
        // new JoystickButton(m_driverStick, IOConstants.kX).onTrue(new TurnToAngleRight(80, m_driveSubsystem));
        // new JoystickButton(m_driverStick, IOConstants.kB).onTrue(new TurnToAngleLeft(80, m_driveSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return m_chooser.getSelected();
    }
}