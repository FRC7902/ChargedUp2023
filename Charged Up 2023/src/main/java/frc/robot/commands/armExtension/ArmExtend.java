// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armExtension;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.ArmExtension;

public class ArmExtend extends CommandBase {
  private ArmExtension m_armExtension;
  private WPI_TalonSRX m_armExtensionMotor;

  /** Creates a new ArmExtend. */
  public ArmExtend(ArmExtension armExtension, WPI_TalonSRX armShoulderLeader) {
    m_armExtension = armExtension;
    m_armExtensionMotor = armShoulderLeader;
    // Use addRequirements() here to declare subsystem dependencies.
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
    m_armExtension.setPower(.35);
    m_armExtension.position++;
    System.out.println(m_armExtension.position);
    System.out.println(m_armExtension.status);

    // int absolutePosition = m_armExtensionMotor.getSensorCollection().getQuadraturePosition();
    // //convert from ticks to degrees
    // double deg = (double)absolutePosition/4096 * 360;
    // System.out.println("POS: " + deg + " " + absolutePosition);
    // double target_sensorUnits = Constants.ArmConstants.kSensorUnitsPerRotation * Constants.ArmConstants.kRotationsToTravel;
    // double adjusted_power = Math.abs((target_sensorUnits-absolutePosition) * 0.0001);
    
    // m_armExtension.set(ControlMode.Position, target_sensorUnits, DemandType.ArbitraryFeedForward, adjusted_power);
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
