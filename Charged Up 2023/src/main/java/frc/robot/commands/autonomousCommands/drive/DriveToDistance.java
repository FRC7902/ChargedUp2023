// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private final double targetDistanceInInches;
  private final PIDController drivePID = new PIDController(DriveConstants.kPDrive, 0, 0);
  /** Creates a new DriveToDistanceNew. */
  public DriveToDistance(double targetDistanceInFeet, DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    targetDistanceInInches = targetDistanceInFeet*12;
    m_DriveSubsystem.resetEncoders();
    drivePID.setTolerance(1); //velocity tolerance is set to infinity because we didn't provide it a value
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = DriveConstants.AutonDriveMultiplier*drivePID.calculate(m_DriveSubsystem.getAvgEncoderDistance(), targetDistanceInInches);
    SmartDashboard.putNumber("PID output",speed);
    
    m_DriveSubsystem.driveRaw(-speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stopMotors();
    m_DriveSubsystem.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint();
  }
}