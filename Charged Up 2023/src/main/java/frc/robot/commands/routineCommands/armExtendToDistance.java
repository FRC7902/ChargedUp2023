// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.ArmExtension;

public class armExtendToDistance extends CommandBase {
  private ArmExtension m_armExtension;
  private double m_distance;
  private int m_direction;
  
  // private WPI_TalonSRX m_armExtensionMotor;

  /** Creates a new ArmExtend. */
  public armExtendToDistance(ArmExtension armExtension, double distance, int direction) {
    m_distance = distance;
    m_direction = direction;
    m_armExtension = armExtension;
    m_armExtension.stopMotor();
    addRequirements(armExtension);
    // m_armExtensionMotor = armShoulderLeader;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armExtension.stopMotor();
    System.out.println("Starting Extension...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armExtension.setPower(m_direction*ArmExtensionConstants.ArmExtensionPower, m_distance);
    System.out.println(m_armExtension.status);

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
