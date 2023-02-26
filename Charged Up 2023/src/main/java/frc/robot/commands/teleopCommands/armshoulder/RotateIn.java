// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.armshoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmShoulder;

public class RotateIn extends CommandBase {

  private ArmShoulder m_armShoulder;
  private WPI_TalonSRX m_armMotor;
  int count = 0;

  /** Creates a new RotateOut. */
  public RotateIn(ArmShoulder armShoulder, WPI_TalonSRX armShoulderLeader) { 
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
    //System.out.println("Arm rotating in..");

    //absolute position gets the location of the arm in ticks (4096 per revolution)
    int absolutePosition = m_armMotor.getSensorCollection().getQuadraturePosition();

    //convert from ticks to degrees
    double deg = (double)absolutePosition/4096 * 360;

    double target_sensorUnits = 0;
    double adjusted_power = Math.abs(absolutePosition * 0.001);
    //double target_sensorUnits = Constants.ArmShoulderConstants.kSensorUnitsPerRotation * Constants.ArmShoulderConstants.kRotationsToTravel;
    //double adjusted_power = Math.abs((target_sensorUnits-absolutePosition) * 0.0001);
    adjusted_power *= (-1)*Constants.ArmShoulderConstants.ArmShoulderRotatePower;

    count++;

    if(count >= 10){
      System.out.println("ROTATING IN POS: " + deg + " " + absolutePosition);
      System.out.println(adjusted_power + " " + target_sensorUnits);
      count = 0;
    }

    //m_armShoulder.setPower(-0.5);
    m_armShoulder.setPower(adjusted_power);

    //m_armShoulder.set(ControlMode.Position, target_sensorUnits, DemandType.ArbitraryFeedForward, adjusted_power);    

    // if(m_armShoulder.atZeroPos()){
    //   m_armShoulder.set(ControlMode.Position, 0);
    // } else {
    //   m_armShoulder.set(ControlMode.Position, target_sensorUnits, DemandType.ArbitraryFeedForward, adjusted_power);
    // }

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
