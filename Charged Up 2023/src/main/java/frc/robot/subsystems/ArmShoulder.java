// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmShoulder{
    //extends TrapezoidProfileSubsystem
    public final static WPI_TalonSRX armShoulderLeader = new WPI_TalonSRX(ArmConstants.ArmShoulderLeaderCAN);
    public final WPI_VictorSPX armShoulderFollower = new WPI_VictorSPX(ArmConstants.ArmShoulderFollowerCAN);
    //private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.ArmSVolts, ArmConstants.ArmGVolts,
    //ArmConstants.ArmVVoltSecondPerRad, ArmConstants.ArmAVoltSecondSquaredPerRad);

  // Need encoder
  // Need limit switch

  /** Creates a new ArmSubsystem. */
  public ArmShoulder() {

    //TRAPEZOID

    // super(new TrapezoidProfile.Constraints(ArmConstants.MaxVelocityRadPerSecond,
    //     ArmConstants.MaxAccelerationRadPerSecSquared), ArmConstants.ArmOffsetRads);
    // armShoulderFollower.follow(armShoulderLeader);
    // armShoulderLeader.setInverted(false);
    // armShoulderFollower.setInverted(InvertType.FollowMaster);
    // armShoulderLeader.setPID(ArmConstants.ArmPosition, 0, 0);

    armShoulderFollower.follow(armShoulderLeader);
    armShoulderLeader.setInverted(false);
    armShoulderLeader.config_kP(Constants.ArmConstants.kSlot_Distanc, Constants.ArmConstants.kGains_Distanc.kP, Constants.ArmConstants.kTimeoutMs);

  }

  public void setPower(double power) {
    armShoulderLeader.set(power);

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

//   void commonLoop() {
//     /* Gamepad processing */
//     double leftYstick = m_driverController.getRawAxis(Constants.IOConstants.kOperatorStick); //Left stick
//     boolean button1 = m_driverController.getRawButton(Constants.IOConstants.kA);	// A-Button
//     boolean button2 = m_driverController.getRawButton(Constants.IOConstants.kB);	// B-Button

//     /* Get Talon/Victor's current output percentage */
//     //double motorOutput = armShoulderLeader.getMotorOutputPercent();

// }

/**
 * This function is called periodically during operator control
 */
// public void teleopPeriodic() {
//     commonLoop();
// }

}
