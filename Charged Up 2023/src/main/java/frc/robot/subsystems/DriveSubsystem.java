// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {
  
  //Motor Controllers
  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(Constants.DriveConstants.DrivetrainLeftLeaderCAN);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(Constants.DriveConstants.DrivetrainLeftFollowerCAN);
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(Constants.DriveConstants.DrivetrainRightLeaderCAN);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(Constants.DriveConstants.DrivetrainRightFollowerCAN);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(left, right);


  public DriveSubsystem() {
    left.setInverted(true);

  }

  public void driveArcade(double xForward, double zRotation) {

    drive.arcadeDrive(xForward, zRotation);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
