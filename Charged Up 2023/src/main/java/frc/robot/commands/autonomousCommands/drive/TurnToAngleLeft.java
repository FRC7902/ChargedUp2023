// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleLeft extends CommandBase {
  private final DriveSubsystem m_DriveSubsystem;
  private final double targetDistanceInInches;
  private final PIDController drivePID = new PIDController(DriveConstants.kPDrive, 0, 0);
  
  public TurnToAngleLeft(double targetDegrees, DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    targetDistanceInInches = (targetDegrees/360)*(DriveConstants.DistanceBetweenWheels*Math.PI);
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
    double speed = MathUtil.clamp(drivePID.calculate(-m_DriveSubsystem.getRightEncoder().getPosition(), targetDistanceInInches),-DriveConstants.ClampingConstant, DriveConstants.ClampingConstant);
    m_DriveSubsystem.turnLeft(-speed);
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
