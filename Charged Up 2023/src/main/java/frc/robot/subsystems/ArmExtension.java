// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmExtensionConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmExtension extends SubsystemBase {

  private final WPI_TalonSRX armExtensionLeader = new WPI_TalonSRX(ArmExtensionConstants.ArmExtensionLeaderCAN);
  private final WPI_VictorSPX armExtensionFollower = new WPI_VictorSPX(ArmExtensionConstants.ArmExtensionFollowerCAN);
  private final Encoder extensionEncoder = new Encoder(ArmExtensionConstants.kEncoderA,ArmExtensionConstants.kEncoderB);
  private final Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
  private double targetExtension;
  public double SDBTargetExtension = 0;
  
  //This encoder connects directly back to 0 and 1 on the roboRIO
  public double percentExtension;
  public int state;

  //Limit Switch
  DigitalInput retractionLimitSwitch = new DigitalInput(ArmExtensionConstants.ZeroPosLimitSwitchDIO);

  //PID Controller
  private final PIDController armExtensionPID = new PIDController(0.5, 0, 0);
  
  //position test;
  public String status = "Off";

  /** Creates a new ArmExtension. */
  public ArmExtension() {
    armExtensionFollower.follow(armExtensionLeader);
    armExtensionLeader.setInverted(false);
    armExtensionFollower.setInverted(InvertType.FollowMaster);
    extensionEncoder.setDistancePerPulse(ArmExtensionConstants.kPC/ArmExtensionConstants.EncoderCPR); //in inches
    extensionEncoder.reset(); //set to zero position
    extensionEncoder.setReverseDirection(true);
    targetExtension = extensionEncoder.getDistance();
    //armExtensionPID.setTolerance(0.5);
    
  //   armExtensionLeader.config_kP(Constants.ArmConstants.kSlot_Distanc, Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kTimeoutMs);
  //   // armShoulderLeader.setPID(Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kGains_Distanc.kI, Constants.ArmConstants.kGains_Distanc.kD);
  // 		/* Motion Magic Configurations */
  //     armExtensionLeader.configMotionAcceleration(2000, Constants.ArmConstants.kTimeoutMs);
  //     armExtensionLeader.configMotionCruiseVelocity(2000, Constants.ArmConstants.kTimeoutMs);
  }

  public void setPower(double power, double distance) {
    // percentExtension = extensionEncoder.getDistance()/distance;

    // if(retractionLimitSwitch.get()){
    //   System.out.println("Hit retraction limit switch");
    //   extensionEncoder.reset();
    //   state = 0;
    // }

    // // if(extensionEncoder.getDistance() < extensionDistanceInInches){
    // //   power = Math.abs(power);
    // // }else if(extensionEncoder.getDistance() > extensionDistanceInInches){
    // //   power = (-1)*Math.abs(power);

    // // }

    // armExtensionLeader.set(power);
    // if(power > 0){
    //   if(percentExtension >= 1){
    //     armExtensionLeader.set(0);
    //     System.out.println("Fully extended.");
    //   }
    //   status = "Extending...";

    // }else if(power < 0){
    //   if(percentExtension <= 1){
    //     armExtensionLeader.set(0); //retraction limit switch is hit so speed is 0
    //   }

    //   // if(extensionEncoder.getDistance() < 1.5){
    //   //   armExtensionLeader.set(0); //retraction limit switch is hit so speed is 0
    //   // }

    //   System.out.println(extensionEncoder.getDistance());
    // }
  }

  public Encoder getEncoder(){
    return extensionEncoder;
  }

  public void stopMotor(){
    armExtensionLeader.stopMotor();
    status = "Off";
  }

  public double getPercentExtension() {
    return extensionEncoder.getDistance() / ArmExtensionConstants.extendedMaxSoftLimitInInches;
  }

  public boolean atZeroPos(){
    return m_debouncer.calculate(retractionLimitSwitch.get());
  }

  public void setTargetPosition(double targetInInches){
    targetExtension = targetInInches;
  }

  @Override
  public void periodic() {
    if(atZeroPos()){
      extensionEncoder.reset(); //set to zero position
    }
    percentExtension = getPercentExtension();

    armExtensionLeader.set(armExtensionPID.calculate(extensionEncoder.getDistance(), targetExtension));


    SmartDashboard.putNumber("% Extension", percentExtension);
    SmartDashboard.putNumber("Target extension", targetExtension);
    SmartDashboard.putNumber("Arm Ext PID Calculate: ", armExtensionPID.calculate(extensionEncoder.get(), SDBTargetExtension));
    SmartDashboard.putBoolean("Arm Limit Switch", retractionLimitSwitch.get());


  }
}