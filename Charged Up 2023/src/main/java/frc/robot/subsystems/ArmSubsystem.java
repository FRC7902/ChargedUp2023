// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends TrapezoidProfileSubsystem {
  private final WPI_TalonSRX LeaderMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderCAN);
  private final WPI_TalonSRX FollowerMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderCAN);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.ArmSVolts, ArmConstants.ArmGVolts, ArmConstants.ArmVVoltSecondPerRad, ArmConstants.ArmAVoltSecondSquaredPerRad);
                   
  

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() { 

    super(new TrapezoidProfile.Constraints(ArmConstants.MaxVelocityRadPerSecond, ArmConstants.MaxAccelerationRadPerSecSquared),ArmConstants.ArmOffsetRads);
        
        
        FollowerMotor.follow(LeaderMotor);
        LeaderMotor.setInverted(false);
        FollowerMotor.setInverted(InvertType.FollowMaster); 
        LeaderMotor.setPID(ArmConstants.kPosition, 0, 0);

  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output


    FollowerMotor.setSetpoint(someSmartMotorController.PIDMode.kPosition, setpoint.position, feedforward / x);
  }
}
