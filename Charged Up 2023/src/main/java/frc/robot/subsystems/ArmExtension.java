// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmExtension extends SubsystemBase {

  private final WPI_TalonSRX armExtensionLeader = new WPI_TalonSRX(ArmConstants.ArmExtensionLeaderCAN);
  private final WPI_VictorSPX armExtensionFollower = new WPI_VictorSPX(ArmConstants.ArmExtensionFollowerCAN);

  /** Creates a new ArmExtension. */
  public ArmExtension() {
    armExtensionFollower.follow(armExtensionLeader);
    armExtensionLeader.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
