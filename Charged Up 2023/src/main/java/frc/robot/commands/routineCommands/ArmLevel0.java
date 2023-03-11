// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.armExtension.ExtendLevel0;
import frc.robot.commands.teleopCommands.armshoulder.RotateLevel0;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLevel0 extends SequentialCommandGroup {
  private final ArmShoulder m_ArmShoulder;
  private final ArmExtension m_Extension;
  public ArmLevel0(ArmShoulder armShoulder, ArmExtension armExtend) {
    m_ArmShoulder = armShoulder;
    m_Extension = armExtend;
    addCommands(
      new ExtendLevel0(m_Extension).withTimeout(1), 
      new RotateLevel0(m_ArmShoulder)
    );
    
  }
}
