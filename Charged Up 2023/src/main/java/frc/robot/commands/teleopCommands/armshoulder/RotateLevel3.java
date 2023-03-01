// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.armshoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmShoulderConstants;
import frc.robot.subsystems.ArmShoulder;

public class RotateLevel3 extends CommandBase {

  private ArmShoulder m_armShoulder;
  int count = 0; // counter for print messages

  /** Creates a new RotateOut. */
  public RotateLevel3(ArmShoulder armShoulder) { 
    m_armShoulder = armShoulder;

    addRequirements(armShoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armShoulder.setTargetPosition(Constants.ArmShoulderConstants.kRotationsToTravel * Constants.ArmShoulderConstants.kSensorUnitsPerRotation);

    System.out.println("!!!!!!!!!! LEVEL 3 Triggered");
  }

  // Called every time the scheduler runs while the command is scheduled.
  // TODO try moving these from execute() to initialize() since they only need to be called once
  @Override
  public void execute() {
    
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
