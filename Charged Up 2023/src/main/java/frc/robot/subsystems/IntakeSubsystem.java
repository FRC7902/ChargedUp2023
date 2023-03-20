// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.IntakeCAN);

  public IntakeSubsystem() {
    intakeMotor.setInverted(false); 
    intakeMotor.enableCurrentLimit(true);
    intakeMotor.configContinuousCurrentLimit(
      15);
    intakeMotor.configPeakCurrentLimit(20);
    intakeMotor.configPeakCurrentDuration(2);
  }

  public void setPower(double power) {
    intakeMotor.set(power);
  }


  public void stopMotor() {
    intakeMotor.stopMotor();
  }

  public void holdCube(){
    setPower(IntakeConstants.HoldingCubeFeedForward);
  }

  public void holdCone(){
    setPower(IntakeConstants.HoldingConeFeedForward);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Current Limit", intakeMotor.getSupplyCurrent());
    // This method will be called once per scheduler run
  }
}
