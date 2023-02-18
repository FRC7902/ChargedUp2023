// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.IntakeCAN);

  public Intake() {
    intakeMotor.setInverted(false);
  }

  public void setPower(double power) {
    intakeMotor.set(power);
  }

  public void stopMotor() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
