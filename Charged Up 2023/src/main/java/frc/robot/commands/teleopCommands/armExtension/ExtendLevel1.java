// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.armExtension;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.ArmExtension;

public class ExtendLevel1 extends CommandBase {
  private ArmExtension m_armExtension;

  /** Creates a new ArmExtend. */
  public ExtendLevel1(ArmExtension armExtension) {
    m_armExtension = armExtension;
    m_armExtension.stopMotor();
    addRequirements(armExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_armExtension.stopMotor();
    m_armExtension.setTargetPosition(ArmExtensionConstants.extendedLevel1SoftLimitInInches);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}