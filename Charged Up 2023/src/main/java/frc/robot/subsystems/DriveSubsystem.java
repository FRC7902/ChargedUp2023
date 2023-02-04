// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {

  // Motor Controllers
  private final CANSparkMax m_leftleader = new CANSparkMax(DriveConstants.DrivetrainLeftLeaderCAN,
      MotorType.kBrushless);
  private final CANSparkMax m_leftfollower = new CANSparkMax(DriveConstants.DrivetrainLeftFollowerCAN,
      MotorType.kBrushless);
  private final CANSparkMax m_rightleader = new CANSparkMax(DriveConstants.DrivetrainRightLeaderCAN,
      MotorType.kBrushless);
  private final CANSparkMax m_rightfollower = new CANSparkMax(DriveConstants.DrivetrainRightFollowerCAN,
      MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftleader, m_leftfollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightleader, m_rightfollower);

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
