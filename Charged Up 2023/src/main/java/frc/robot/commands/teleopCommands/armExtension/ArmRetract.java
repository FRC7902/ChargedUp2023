// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.armExtension;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.ArmExtension;

public class ArmRetract extends CommandBase {
  private ArmExtension m_armExtension;

  /** Creates a new Retract. */
  public ArmRetract(ArmExtension armExtension) {
    m_armExtension = armExtension;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armExtension.stopMotor();
    System.out.println("Retracting Extension...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_armExtension.setPower(-1*Constants.ArmExtensionConstants.ArmExtensionPower, 0);
    System.out.println(m_armExtension.status);
  }

  // int absolutePosition =
  // m_armExtensionMotor.getSensorCollection().getQuadraturePosition();
  // //convert from ticks to degrees
  // double deg = (double)absolutePosition/4096 * 360;
  // System.out.println("POS: " + deg + " " + absolutePosition);
  // double target_sensorUnits = Constants.ArmConstants.kSensorUnitsPerRotation *
  // Constants.ArmConstants.kRotationsToTravel;
  // double adjusted_power = Math.abs((target_sensorUnits-absolutePosition) *
  // 0.0001);

  // m_armExtension.set(ControlMode.Position, target_sensorUnits,
  // DemandType.ArbitraryFeedForward, -1*adjusted_power);
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
