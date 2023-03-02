// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmShoulder;

public class homing extends CommandBase {
  private ArmExtension m_armExtension;
  private ArmShoulder m_armShoulder;

  /** Creates a new homing. */
  public homing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!m_armShoulder.atZeroPos()){
      m_armShoulder.setPower(-0.2);
    }

    if(!m_armExtension.atZeroPos()){
      m_armExtension.setPower(-0.2);
    }

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
