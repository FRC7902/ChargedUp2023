// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.GainConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleLeft extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngleLeft(double targetDegrees, DriveSubsystem m_DriveSubsystem) { 
    super(
        new PIDController(GainConstants.kGains_Turning.kP, GainConstants.kGains_Turning.kI, GainConstants.kGains_Distanc.kD),
        //in inches
        () -> m_DriveSubsystem.getLeftEncoder().getPosition(),
        // in inches
        () -> (targetDegrees/360)*(DriveConstants.DistanceBetweenWheels*Math.PI),
        output -> {
          double targetDistanceInInches = DriveConstants.DistanceBetweenWheels*Math.PI;
          double adjustment = output/(targetDistanceInInches);
          m_DriveSubsystem.turnLeft(adjustment);
        }, m_DriveSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
