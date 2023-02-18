// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//command imports
import frc.robot.commands.armshoulder.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.armExtension.*;
import frc.robot.commands.drive.*;

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
  private final ArmShoulder m_ArmShoulder = new ArmShoulder();
  private final ArmExtension m_ArmExtension = new ArmExtension();
  private final Intake m_intake = new Intake();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  //THE FIRST CONTOLLER PLUGGED IN CONTROLS THE DRIVETRAIN, THE SECOND CONTROLLER PLUGGED IN CONTROLS THE ARM/INTAKE
  private final Joystick m_driverStick = new Joystick(Constants.IOConstants.kDriverStick);
  private final XboxController m_driverController = new XboxController(Constants.IOConstants.kDriverStick);//should be kOperatorStick

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

    //SHOULDER BINDINGS
    new JoystickButton(m_driverController, Constants.IOConstants.kA).whileTrue(new RotateOut(m_ArmShoulder, ArmShoulder.armShoulderLeader));//kA
    new JoystickButton(m_driverController, Constants.IOConstants.kB).whileTrue(new RotateIn(m_ArmShoulder, ArmShoulder.armShoulderLeader));//kB
    new JoystickButton(m_driverController, Constants.IOConstants.kA).onFalse(new Hold(m_ArmShoulder, ArmShoulder.armShoulderLeader, Constants.ArmShoulderConstants.ArmShoulderHold));//kA
    
    //EXTENSION BINDINGS
    new JoystickButton(m_driverController, Constants.IOConstants.kY).whileTrue(new ArmExtend(m_ArmExtension, ArmExtension.armExtensionLeader)); //kY
    new JoystickButton(m_driverController, Constants.IOConstants.kX).whileTrue(new ArmRetract(m_ArmExtension, ArmExtension.armExtensionLeader)); //kX
    new JoystickButton(m_driverController, Constants.IOConstants.kY).onFalse(new ArmHold(m_ArmExtension, ArmExtension.armExtensionLeader, Constants.ArmShoulderConstants.ArmShoulderHold));//kY
    
    //INTAKE BINDINGS
    new JoystickButton(m_driverController, Constants.IOConstants.kLB).whileTrue(new directionA(m_intake));//kLB
    new JoystickButton(m_driverController, Constants.IOConstants.kRB).whileTrue(new directionB(m_intake));//kRB

    //TESTERS

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  //   An example command will be run in autonomous
  return m_chooser.getSelected();
  }
}
