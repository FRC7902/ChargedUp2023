// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  //Motor Controllers
  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(Constants.DriveConstants.kLeftLeaderCAN);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(Constants.DriveConstants.kLeftFollowerCAN);
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(Constants.DriveConstants.kRightLeaderCAN);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(Constants.DriveConstants.kRightFollowerCAN);

  public DriveSubsystem() {
    m_rightFollower.follow(m_rightLeader);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_leftFollower.follow(m_leftLeader);
    m_leftFollower.setInverted(InvertType.FollowMaster);

    m_leftLeader.setInverted(false);
    m_rightLeader.setInverted(true);

  }

  public void driveArcade(double xForward, double zRotation) {

    drive.arcadeDrive(xForward, zRotation);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
