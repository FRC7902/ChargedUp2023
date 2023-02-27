// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ArmShoulderConstants;
import frc.robot.commands.routineCommands.armShoulderRotateToAngle;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmShoulder extends SubsystemBase{
  public final static WPI_TalonSRX armShoulderLeader = new WPI_TalonSRX(ArmShoulderConstants.ArmShoulderLeaderCAN);
  public final static WPI_VictorSPX armShoulderFollower = new WPI_VictorSPX(ArmShoulderConstants.ArmShoulderFollowerCAN);

  // Encoder

  // Need limit switch

  /** Creates a new ArmSubsystem. */
  public ArmShoulder() {
    armShoulderFollower.follow(armShoulderLeader);
    armShoulderLeader.setInverted(false);
    armShoulderFollower.setInverted(InvertType.FollowMaster);

    // armShoulderLeader.config_kP(Constants.GainConstants.kSlot_Distanc, Constants.GainConstants.kGains_Distanc.kP,
    //     Constants.ArmShoulderConstants.kTimeoutMs);
    // armShoulderLeader.configMotionAcceleration(2000, Constants.ArmShoulderConstants.kTimeoutMs);
    // armShoulderLeader.configMotionCruiseVelocity(2000, Constants.ArmShoulderConstants.kTimeoutMs);

    // Encoder
    armShoulderLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
        0);

    armShoulderLeader.setInverted(false);
    armShoulderLeader.setSensorPhase(true);

    // limit switch
    armShoulderLeader.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
        LimitSwitchNormal.NormallyOpen);

    armShoulderLeader.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);

  }

  public int getLimitSwitch(){
    return armShoulderLeader.isRevLimitSwitchClosed();
  }



  public void setPower(double power) {
    armShoulderLeader.set(power);

  }

  // need configure the encoder 2:1 ratio

  public void setPosition(double demand0, double demand1) {
    armShoulderLeader.set(ControlMode.Position, demand0, DemandType.ArbitraryFeedForward, 0);

    // if statements needed for testing
  }

  public void set(ControlMode mode, double value){
    armShoulderLeader.set(mode, value);
  }

  public double getFollowerPower(){
    return armShoulderFollower.get();
  }

  public double getLeaderPower(){
    return armShoulderLeader.get();
  }

  public double getEncoderPos(){ //needs testing
    return armShoulderLeader.getSelectedSensorPosition()*2;
  }

  public boolean atZeroPos(){
    return armShoulderLeader.isRevLimitSwitchClosed() == 0; //switch is open
  }

  public void stopMotor() {
    armShoulderLeader.stopMotor();
  }

  public double getPosition () {
    //absolute position gets the location of the arm in ticks (4096 per revolution)
    return armShoulderLeader.getSelectedSensorPosition();
  }

  // METHOD - GET NEW POSITION



  /**
   * This function is called periodically during operator control
   */

  @Override
  public void periodic() {

    if(armShoulderLeader.isRevLimitSwitchClosed() == 1){
      System.out.println("LIMIT SWITCH TRIGGERED");
      armShoulderLeader.getSensorCollection().setQuadraturePosition(0, 0);
    }

    // Position control to current position variable


  }

}
