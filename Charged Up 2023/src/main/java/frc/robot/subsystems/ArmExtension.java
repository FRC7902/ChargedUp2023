// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmExtensionConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmExtension extends SubsystemBase {

  private final WPI_TalonSRX armExtensionLeader = new WPI_TalonSRX(ArmExtensionConstants.ArmExtensionLeaderCAN);
  private final WPI_VictorSPX armExtensionFollower = new WPI_VictorSPX(ArmExtensionConstants.ArmExtensionFollowerCAN);
  private final Encoder extensionEncoder = new Encoder(ArmExtensionConstants.kEncoderA,ArmExtensionConstants.kEncoderB);
  //This encoder connects directly back to 0 and 1 on the roboRIO
  public double currentPercentExtension;
  public double targetPercentExtension;
  
  int counter = 0;

  //Limit Switch
  DigitalInput retractionLimitSwitch = new DigitalInput(ArmExtensionConstants.ZeroPosLimitSwitchDIO);
  
  //position test;
  public String status = "Off";

  /** Creates a new ArmExtension. */
  public ArmExtension() {
    armExtensionFollower.follow(armExtensionLeader);
    armExtensionLeader.setInverted(false);
    armExtensionFollower.setInverted(InvertType.FollowMaster);
    extensionEncoder.setDistancePerPulse(ArmExtensionConstants.kPC/ArmExtensionConstants.EncoderCPR); //in inches
    extensionEncoder.reset(); //set to zero position
    targetPercentExtension = extensionEncoder.getDistance()/Constants.ArmExtensionConstants.extendedMaxSoftLimitInInches;
    
  //   armExtensionLeader.config_kP(Constants.ArmConstants.kSlot_Distanc, Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kTimeoutMs);
  //   // armShoulderLeader.setPID(Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kGains_Distanc.kI, Constants.ArmConstants.kGains_Distanc.kD);
  // 		/* Motion Magic Configurations */
  //     armExtensionLeader.configMotionAcceleration(2000, Constants.ArmConstants.kTimeoutMs);
  //     armExtensionLeader.configMotionCruiseVelocity(2000, Constants.ArmConstants.kTimeoutMs);
  }

  public double getPercentExtension(){
    return extensionEncoder.getDistance()/Constants.ArmExtensionConstants.extendedMaxSoftLimitInInches;
  }

  public void setTargetPercentExtension(double percentExtension){
    targetPercentExtension = percentExtension;
  }

  public Encoder getEncoder(){
    return extensionEncoder;
  }

  public void stopMotor(){
    armExtensionLeader.stopMotor();
    status = "Off";
  }

  // public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
  //   armExtensionLeader.set(mode, demand0, demand1Type, demand1);
  // }

  @Override
  public void periodic() {
    if(retractionLimitSwitch.get()){
      System.out.println("Hit retraction limit switch");
      extensionEncoder.reset();
    }

    double adjusted_power;

    adjusted_power = targetPercentExtension - currentPercentExtension;

    armExtensionLeader.set(adjusted_power);

    double currentPercentProgress = currentPercentExtension/targetPercentExtension;

    counter++;

    if(counter >= 10){
      System.out.println(extensionEncoder.getDistance());
    }

    if(adjusted_power > 0){
      if(currentPercentProgress >= 1){
        armExtensionLeader.set(0);
        System.out.println("Hit max");
      }

    }else if(adjusted_power < 0){
      if(currentPercentProgress <= 1){
        armExtensionLeader.set(0); //retraction limit switch is hit so speed is 0
        System.out.println("Hit min");
      }
    }

    // This method will be called once per scheduler run
  }
}
