// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCone extends CommandBase {

  private final IntakeSubsystem m_intake;
  private final double m_mulitplier;

  /** Creates a new ShootCone, the multiplayer should allow for flexible speeds */
  public ShootCone(IntakeSubsystem intake, double multiplier) { 
    m_intake = intake;
    m_mulitplier = multiplier;
  }

  //overloaded for auton
  public ShootCone(IntakeSubsystem intake) { 
    m_intake = intake;
    m_mulitplier = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopMotor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPower(IntakeConstants.ShootConeSpeed*m_mulitplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
