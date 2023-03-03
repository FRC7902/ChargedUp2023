// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.armExtension;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.ArmExtension;

public class ExtendLevel3 extends CommandBase {
  private ArmExtension m_armExtension;

  /** Creates a new Retract. */
  public ExtendLevel3(ArmExtension armExtension) {
    m_armExtension = armExtension;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armExtension.setTargetPercentExtension(ArmExtensionConstants.kLevel3Percentage);
    System.out.println("Set extension level 3.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
