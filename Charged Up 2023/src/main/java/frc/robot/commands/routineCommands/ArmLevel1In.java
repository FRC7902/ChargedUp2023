// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.teleopCommands.armExtension.ExtendLevel1;
import frc.robot.commands.teleopCommands.armshoulder.RotateLevel1;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLevel1In extends SequentialCommandGroup {
  /** Creates a new armExtendToLow. */

  private final ArmShoulder m_ArmShoulder;
  private final ArmExtension m_ArmExtension;

  public ArmLevel1In(ArmShoulder armShoulder, ArmExtension armExtend) {
    m_ArmShoulder = armShoulder;
    m_ArmExtension = armExtend;
    addCommands(
      new ExtendLevel1(m_ArmExtension).withTimeout(Constants.ArmExtensionConstants.ExtensionBufferTimeInSeconds),
      new RotateLevel1(m_ArmShoulder)
    );
  }

}
