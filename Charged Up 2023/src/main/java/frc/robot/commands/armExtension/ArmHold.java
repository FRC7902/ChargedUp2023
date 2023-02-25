// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armExtension;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.ArmExtension;

public class ArmHold extends CommandBase {

  private ArmExtension m_armExtension;
  private double m_power;


  /** Creates a new ArmHold. */
  public ArmHold(ArmExtension armExtension,  double feedForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armExtension = armExtension;
    m_power = feedForward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //stops motor before running
    m_armExtension.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armExtension.setPower(m_power);
    m_armExtension.status = "Off";
    
    // //absolute position gets the location of the arm in ticks (4096 per revolution)
    // int absolutePosition = m_armExtensionMotor.getSensorCollection().getQuadraturePosition();
    // //convert from ticks to degrees
    // double deg = (double)absolutePosition/4096 * 360;
    // System.out.println("POS: " + deg + " " + absolutePosition);
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
