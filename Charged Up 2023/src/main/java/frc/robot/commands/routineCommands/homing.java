// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.Constants.ArmShoulderConstants;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmShoulder;

public class homing extends CommandBase {
  private final ArmExtension m_armExtension;
  private final ArmShoulder m_armShoulder;

  /** Creates a new homing. */
  public homing(ArmExtension armExtension, ArmShoulder armShoulder) {
    m_armExtension = armExtension;
    m_armShoulder = armShoulder;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_armShoulder.atZeroPos()) {
      m_armShoulder.setPower(-1 * ArmShoulderConstants.shoulderHomingPower);
    }

    if (!m_armExtension.atZeroPos()) {
      m_armExtension.setPower(-1 * ArmExtensionConstants.extensionHomingPower);
    }

    System.out.println("Running.");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Done");
    return (m_armExtension.atZeroPos() && m_armShoulder.atZeroPos());
  }
}
