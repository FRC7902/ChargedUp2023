// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceSetPower extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private AutoBalance m_AutoBalance;

  /** Creates a new SlowDriveForward. */
  public AutoBalanceSetPower(DriveSubsystem driveSubsystem, AutoBalance autoBalance) {
    m_driveSubsystem = driveSubsystem;
    m_AutoBalance = autoBalance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = m_AutoBalance.AutoBalancingSpeed();
    m_driveSubsystem.driveRaw(speed);
    SmartDashboard.putNumber("Autobal Speed", speed);
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
