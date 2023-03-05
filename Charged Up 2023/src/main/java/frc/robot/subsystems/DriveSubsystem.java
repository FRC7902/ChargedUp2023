// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  // Encoders
  private final RelativeEncoder m_leftEncoder = m_leftleader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightleader.getEncoder();

  public DriveSubsystem() {
    left.setInverted(true);
    m_leftleader.setSmartCurrentLimit(DriveConstants.SoftwareCurrentLimit);
    m_rightleader.setSmartCurrentLimit(DriveConstants.SoftwareCurrentLimit);
    m_leftleader.setOpenLoopRampRate(2);
    m_rightleader.setOpenLoopRampRate(2);
    resetEncoders();

    // tells how far you travelled in inches. NOTE: RIGHT IS NEGATIVE WHEN DRIVING FORWARD
    m_leftEncoder
        .setPositionConversionFactor(DriveConstants.OutputGearRatio * DriveConstants.WheelCircumferenceInInches);
    m_rightEncoder
        .setPositionConversionFactor(DriveConstants.OutputGearRatio * DriveConstants.WheelCircumferenceInInches);

  }

  double tester = 0; // this is used to prevent the values from flashing too quickly across the
                     // screeen

  public void driveArcade(double xForward, double zRotation) {
    if (tester < 50) { // to slow the output for testing purposes
      tester++;
    } else {
      // Note from Jane: I fear that there may be a bug here. Regardless of whether I
      // put -1 as the right position conversion factor, it's always negative when
      // moving forward. Just be aware of this if it can't be fixed.
      //System.out.println("Position left: " + m_leftEncoder.getPosition());
      //System.out.println("Position right: " + (-1)*m_rightEncoder.getPosition());
      tester = 0;
    }

    drive.arcadeDrive(xForward, zRotation);

  }

  public void driveRaw(double left, double right) {
    this.left.set(left);
    this.right.set(right);
  }

  public void turnLeft(double amount){ 
    left.set(amount);
    right.set(-amount);
  }

  public void turnRight(double amount){ 
    right.set(amount);
    left.set(-amount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  public double getAvgEncoderDistance(){
    return (m_leftEncoder.getPosition() - m_rightEncoder.getPosition())/2;
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

}
