// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armExtension;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.ArmExtension;

public class ArmHold extends CommandBase {

  private ArmExtension m_ArmExtension;
  private WPI_TalonSRX m_armExtensionMotor;
  private double power;


  /** Creates a new ArmHold. */
  public ArmHold(ArmExtension armExtension, WPI_TalonSRX armExtensionLeader, double pow) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmExtension = armExtension;
    m_armExtensionMotor = armExtensionLeader;
    power = pow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //stops motor before running
    m_ArmExtension.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmExtension.setPower(power);

    //absolute position gets the location of the arm in ticks (4096 per revolution)
    int absolutePosition = m_armExtensionMotor.getSensorCollection().getQuadraturePosition();
    //convert from ticks to degrees
    double deg = (double)absolutePosition/4096 * 360;
    System.out.println("POS: " + deg + " " + absolutePosition);
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
