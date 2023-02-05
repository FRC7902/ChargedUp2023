// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armshoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmShoulderBasic;

public class RotateOut extends CommandBase {

  //private ArmShoulder m_armShoulder;
  private ArmShoulderBasic m_armShoulder;
  private WPI_TalonSRX m_armMotor;

  /** Creates a new RotateOut. */
  public RotateOut(ArmShoulderBasic armShoulder, WPI_TalonSRX armShoulderLeader) { 
    m_armShoulder = armShoulder;
    m_armMotor = armShoulderLeader;

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
    //System.out.println("Arm rotating out..");

    //int absolutePosition = m_armShoulder.getSensorCollection().getPulseWidthPosition();


    int absolutePosition = m_armMotor.getSensorCollection().getQuadraturePosition();
    absolutePosition &= 0xFFF;

    double deg = (double)absolutePosition/4096 * 360;

    System.out.println("POS: " + deg + " " + absolutePosition);
    //targetPositionRotations = (leftYstick + 0.1) * 10.0 * 4096;
    //m_armShoulder.setLocation(ControlMode.Position, 10.0*4096);

    if(true){//deg > 0
      m_armShoulder.setPower(ArmConstants.ArmShoulderRotateOut);
    }

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
