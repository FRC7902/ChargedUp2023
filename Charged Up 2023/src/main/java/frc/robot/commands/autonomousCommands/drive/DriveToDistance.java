// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.GainConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends PIDCommand {
  private final DriveSubsystem m_DriveSubsystem;

  public DriveToDistance(double targetDistanceInFeet, DriveSubsystem m_driveSubsytem) {
    super(
        // The controller that the command will use
        new PIDController(GainConstants.kGains_Distanc.kP, GainConstants.kGains_Distanc.kI, GainConstants.kGains_Distanc.kD), 
        // This should return the measurement
        m_driveSubsytem::getAvgEncoderDistance, //distance to be measured in inches
        // This should return the setpoint (can also be a constant)
        targetDistanceInFeet*12, //target distance in inches
        // This uses the output
        output -> { //Going off the interpretation that output = error between target and current position
          double adjustment = output/(targetDistanceInFeet*12);
          m_driveSubsytem.driveRaw(adjustment, adjustment);
        }, m_driveSubsytem);

        m_DriveSubsystem = m_driveSubsytem;
        getController().setTolerance(1);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted){
    m_DriveSubsystem.resetEncoders();
  }
}
