// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.teleopCommands.armExtension.ExtendLevel2;
import frc.robot.commands.teleopCommands.armshoulder.RotateLevel2;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLevel2Sequential extends SequentialCommandGroup {

  private final ArmShoulder m_ArmShoulder;
  private final ArmExtension m_ArmExtension;
  /** Creates a new armExtendToLow. */
  public ArmLevel2Sequential(ArmShoulder armShoulder, ArmExtension armExtend) {
    m_ArmShoulder = armShoulder;
    m_ArmExtension = armExtend;
    addCommands(
        new RotateLevel2(m_ArmShoulder).withTimeout(Constants.ArmShoulderConstants.ShoulderBufferTimeInSeconds),
        new ExtendLevel2(m_ArmExtension)
    );

  }
}
