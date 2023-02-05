// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armshoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmShoulderBasic;

public class RotateOut extends CommandBase {

  //private ArmShoulder m_armShoulder;
  private ArmShoulderBasic m_armShoulder;

  /** Creates a new RotateOut. */
  public RotateOut(ArmShoulderBasic armShoulder) {
    m_armShoulder = armShoulder;
    //addRequirements(armShoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armShoulder.stopMotor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Arm rotating out..");
    m_armShoulder.setPower(ArmConstants.ArmShoulderRotateOut);

    //targetPositionRotations = (leftYstick + 0.1) * 10.0 * 4096;
    //m_armShoulder.setLocation(ControlMode.Position, 10.0*4096);

  }

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
