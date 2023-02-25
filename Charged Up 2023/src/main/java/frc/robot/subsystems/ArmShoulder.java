// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ArmShoulderConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmShoulder {
  // extends TrapezoidProfileSubsystem
  public final static WPI_TalonSRX armShoulderLeader = new WPI_TalonSRX(ArmShoulderConstants.ArmShoulderLeaderCAN);
  public final WPI_VictorSPX armShoulderFollower = new WPI_VictorSPX(ArmShoulderConstants.ArmShoulderFollowerCAN);

  // Encoder

  // Need limit switch

  /** Creates a new ArmSubsystem. */
  public ArmShoulder() {
    armShoulderFollower.follow(armShoulderLeader);
    armShoulderLeader.setInverted(false);
    armShoulderFollower.setInverted(InvertType.FollowMaster);

    armShoulderLeader.config_kP(Constants.GainConstants.kSlot_Distanc, Constants.GainConstants.kGains_Distanc.kP,
        Constants.ArmShoulderConstants.kTimeoutMs);
    armShoulderLeader.configMotionAcceleration(2000, Constants.ArmShoulderConstants.kTimeoutMs);
    armShoulderLeader.configMotionCruiseVelocity(2000, Constants.ArmShoulderConstants.kTimeoutMs);

    // Encoder
    armShoulderLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0,
        Constants.ArmShoulderConstants.kTimeoutMs);

    // limit switch
    armShoulderLeader.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
        LimitSwitchNormal.NormallyClosed);
  }

  public void setPower(double power) {
    armShoulderLeader.set(power);

  }

  // need configure the 2:1 ratio

  public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
    System.out.println(armShoulderLeader.getSelectedSensorPosition()); // needs testing
    armShoulderLeader.set(mode, demand0, demand1Type, demand1);

    // if statements needed for testing
  }

  public void set(ControlMode mode, double value){
    armShoulderLeader.set(mode, value);
  }

  public boolean atZeroPos(){
    return armShoulderLeader.isRevLimitSwitchClosed() == 0; //switch is open
  }

  public void stopMotor() {
    armShoulderLeader.stopMotor();
  }

  /**
   * This function is called periodically during operator control
   */
  public void teleopPeriodic() {
  }

}
