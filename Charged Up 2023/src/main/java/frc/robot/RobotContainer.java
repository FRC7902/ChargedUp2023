// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.autonomousCommands.drive.*;
import frc.robot.commands.routineCommands.homing;
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
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmExtension m_ArmExtension = new ArmExtension();
  private final ArmShoulder m_ArmShoulder = new ArmShoulder(m_ArmExtension);
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // Auton commands:
  private final DriveToDistance m_DriveToDistance = new DriveToDistance(2, m_driveSubsystem);
  private final TurnToAngleR m_turnToAngleLeft = new TurnToAngleR(30, m_driveSubsystem);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  // THE FIRST CONTOLLER PLUGGED IN CONTROLS THE DRIVETRAIN, THE SECOND CONTROLLER
  // PLUGGED IN CONTROLS THE ARM/INTAKE
  private final XboxController m_driverStick = new XboxController(IOConstants.kDriverStick);
  private final XboxController m_operatorStick = new XboxController(IOConstants.kOperatorStick);// should be
                                                                                                // kOperatorStick

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.driveArcade(m_driverStick.getRawAxis(Constants.IOConstants.kLY),
                m_driverStick.getRawAxis(Constants.IOConstants.kRX)),
            m_driveSubsystem));

    // AUTON TESTING
    // m_chooser.setDefaultOption("Drive to Distance", m_DriveToDistance);
    // m_chooser.addOption("Turn 30 degrees left", m_turnToAngleLeft);
    // SmartDashboard.putData(m_chooser);
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
    // new JoystickButton(m_driverStick, IOConstants.kLB).whileTrue(new SlowDriveForward(m_driveSubsystem));
    // new JoystickButton(m_driverStick, IOConstants.kLT).whileTrue(new SlowDriveBackward(m_driveSubsystem));

    // Homing
    new JoystickButton(m_operatorStick, Constants.IOConstants.kSTART)
        .toggleOnTrue(new homing(m_ArmExtension, m_ArmShoulder));

    // SHOULDER BINDINGS
    new JoystickButton(m_operatorStick, IOConstants.kA).onTrue(new RotateLevel0(m_ArmShoulder));
    new JoystickButton(m_operatorStick, IOConstants.kB).onTrue(new RotateLevel1(m_ArmShoulder));
    new JoystickButton(m_operatorStick, IOConstants.kY).onTrue(new RotateLevel2(m_ArmShoulder));
    new JoystickButton(m_operatorStick, IOConstants.kX).onTrue(new RotateLevel3(m_ArmShoulder));

    //EXTENSION BINDINGS
    new POVButton(m_operatorStick, 0).onTrue(new ExtendLevel0(m_ArmExtension));
    new POVButton(m_operatorStick, 270).onTrue(new ExtendLevel1(m_ArmExtension));
    new POVButton(m_operatorStick, 180).onTrue(new ExtendLevel2(m_ArmExtension));
    new POVButton(m_operatorStick, 90).onTrue(new ExtendLevel3(m_ArmExtension));


    // COMPOUND ARM MOVEMENT BINDINGS

    // new JoystickButton(m_operatorStick, IOConstants.kA).onTrue(new
    // ArmLevel0(m_ArmShoulder, m_ArmExtension));
    // new JoystickButton(m_operatorStick, IOConstants.kB).onTrue(new
    // ArmLevel1(m_ArmShoulder, m_ArmExtension));
    // new JoystickButton(m_operatorStick, IOConstants.kY).onTrue(new
    // ArmLevel2(m_ArmShoulder, m_ArmExtension));
    // new JoystickButton(m_operatorStick, IOConstants.kX).onTrue(new
    // ArmLevel3(m_ArmShoulder, m_ArmExtension));

    // INTAKE BINDINGS

    // Cone stuff (left hand)
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileTrue(new suckCone(m_intake));// kLB
    //new JoystickButton(m_operatorStick, IOConstants.kLT).whileTrue(new shootCone(m_intake)); // kLT
    new JoystickButton(m_operatorStick, IOConstants.kLB).onFalse(new IntakeStop(m_intake)); // kLB
   // new JoystickButton(m_operatorStick, IOConstants.kLT).onFalse(new IntakeStop(m_intake)); // kLT

    // Cube stuff (right hand)
    new JoystickButton(m_operatorStick, IOConstants.kRB).whileTrue(new suckCube(m_intake));// kRB
   // new JoystickButton(m_operatorStick, IOConstants.kRT).whileTrue(new shootCube(m_intake)); // kRT
    new JoystickButton(m_operatorStick, IOConstants.kRB).onFalse(new IntakeStop(m_intake));// kRB
    //new JoystickButton(m_operatorStick, IOConstants.kRT).onFalse(new IntakeStop(m_intake)); // kRT

    // ROUTINE BINDINGS
    // new JoystickButton(m_operatorStick, IOConstants.kMENU).onTrue(new
    // armExtendShoulderToHigh(m_ArmShoulder, null, m_ArmExtension))

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