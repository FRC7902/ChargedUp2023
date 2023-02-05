// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmShoulder{
    //extends TrapezoidProfileSubsystem
    public final static WPI_TalonSRX armShoulderLeader = new WPI_TalonSRX(ArmConstants.ArmShoulderLeaderCAN);
    public final WPI_VictorSPX armShoulderFollower = new WPI_VictorSPX(ArmConstants.ArmShoulderFollowerCAN);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.ArmSVolts, ArmConstants.ArmGVolts,
    ArmConstants.ArmVVoltSecondPerRad, ArmConstants.ArmAVoltSecondSquaredPerRad);

  // Need encoder
  // Need limit switch

  /** Creates a new ArmSubsystem. */
  public ArmShoulder() {

    //TRAPEZOID
    // super(new TrapezoidProfile.Constraints(ArmConstants.MaxVelocityRadPerSecond,
    //     ArmConstants.MaxAccelerationRadPerSecSquared);

    armShoulderFollower.follow(armShoulderLeader);
    armShoulderLeader.setInverted(false);
    armShoulderFollower.setInverted(InvertType.FollowMaster);
    
    armShoulderLeader.config_kP(Constants.ArmConstants.kSlot_Distanc, Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kTimeoutMs);
    // armShoulderLeader.setPID(Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kGains_Distanc.kI, Constants.ArmConstants.kGains_Distanc.kD);
  		/* Motion Magic Configurations */
      armShoulderLeader.configMotionAcceleration(2000, Constants.ArmConstants.kTimeoutMs);
      armShoulderLeader.configMotionCruiseVelocity(2000, Constants.ArmConstants.kTimeoutMs);
  
  }

  public void setPower(double power) {
    armShoulderLeader.set(power);

    // if statements needed for testing
  }

  public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
    armShoulderLeader.set(mode, demand0, demand1Type, demand1);

    // if statements needed for testing
  }

  public void stopMotor() {
    armShoulderLeader.stopMotor();
  }
  
  //TRAPEZIOD
  // @Override
  // protected void useState(TrapezoidProfile.State state) {
  //   double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  //   // Add the feedforward to the PID output to get the motor output
  //   armShoulderLeader.setSetpoint(WPI_TalonSRX.PIDMode.ArmPosition, setpoint.position, feedforward / x);
  // }

/**
 * This function is called periodically during operator control
 */
public void teleopPeriodic() {}

}
