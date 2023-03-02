// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.ArmExtension;

public class armExtendToDistance extends CommandBase {
  private ArmExtension m_armExtension;
  private double m_targetExtension;

  /** Creates a new Retract. */
  public armExtendToDistance(ArmExtension armExtension, double lengthInInches) {
    m_armExtension = armExtension;
    m_targetExtension = lengthInInches/ArmExtensionConstants.extendedMaxSoftLimitInInches;
    addRequirements(m_armExtension);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armExtension.setTargetPercentExtension(m_targetExtension);
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
