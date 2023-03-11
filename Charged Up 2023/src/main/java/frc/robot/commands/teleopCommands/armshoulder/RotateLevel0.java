// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.armshoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmShoulderConstants;
import frc.robot.subsystems.ArmShoulder;

public class RotateLevel0 extends CommandBase {

  private ArmShoulder m_armShoulder;
  int count = 0;


  /** Creates a new RotateOut. */
  public RotateLevel0(ArmShoulder armShoulder) { 
    m_armShoulder = armShoulder;
    addRequirements(armShoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armShoulder.setTargetPosition(ArmShoulderConstants.kLevel0EncoderTicks);
    System.out.println("Level 0 Triggered");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armShoulder.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
