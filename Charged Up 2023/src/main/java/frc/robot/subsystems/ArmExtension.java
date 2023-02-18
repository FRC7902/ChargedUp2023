// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmExtensionConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmExtension extends SubsystemBase {

  public static final WPI_TalonSRX armExtensionLeader = new WPI_TalonSRX(15);//should be: ArmConstants.ArmExtensionLeaderCAN
  public static final WPI_VictorSPX armExtensionFollower = new WPI_VictorSPX(ArmExtensionConstants.ArmExtensionFollowerCAN);

  public String status = "Off";

  /** Creates a new ArmExtension. */
  public ArmExtension() {


    armExtensionFollower.follow(armExtensionLeader);
    armExtensionLeader.setInverted(false);
    armExtensionFollower.setInverted(InvertType.FollowMaster);
    
    armExtensionLeader.config_kP(Constants.ArmShoulderConstants.kSlot_Distanc, Constants.ArmShoulderConstants.kGains_Distanc.kP, Constants.ArmShoulderConstants.kTimeoutMs);
    // armShoulderLeader.setPID(Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kGains_Distanc.kI, Constants.ArmConstants.kGains_Distanc.kD);
  		/* Motion Magic Configurations */
      armExtensionLeader.configMotionAcceleration(2000, Constants.ArmShoulderConstants.kTimeoutMs);
      armExtensionLeader.configMotionCruiseVelocity(2000, Constants.ArmShoulderConstants.kTimeoutMs);
  }

  public void setPower(double power) {
    armExtensionLeader.set(power);
    if(power > 0){
      status = "Extending...";
    }else if(power < 0){
      status = "Retracting...";
    }
  }

  public void stopMotor(){
    armExtensionLeader.stopMotor();
    status = "Off";
  }

  public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
    armExtensionLeader.set(mode, demand0, demand1Type, demand1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Extension Status: ", status);
  }
}
