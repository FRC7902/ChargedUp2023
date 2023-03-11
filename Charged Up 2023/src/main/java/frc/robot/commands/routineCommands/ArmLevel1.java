// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.teleopCommands.armExtension.ExtendLevel1;
import frc.robot.commands.teleopCommands.armshoulder.RotateLevel1;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLevel1 extends SequentialCommandGroup {
  /** Creates a new armExtendToLow. */

  private final ArmShoulder m_ArmShoulder;
  private final ArmExtension m_ArmExtension;

  public ArmLevel1(ArmShoulder armShoulder, ArmExtension armExtend) {
    m_ArmShoulder = armShoulder;
    m_ArmExtension = armExtend;
    double currentArmAngle = m_ArmShoulder.getPosition()*2;
    SmartDashboard.putNumber("ArmLevel1 Current Arm Angle",currentArmAngle);
    SmartDashboard.putNumber("ArmLevel1 Target",Constants.ArmShoulderConstants.kLevel1EncoderTicks);

    if(currentArmAngle < Constants.ArmShoulderConstants.kLevel1EncoderTicks){
      SmartDashboard.putString("Order","Rotating first");
      addCommands(
        new RotateLevel1(m_ArmShoulder).withTimeout(1), new ExtendLevel1(m_ArmExtension)
      );
    }else{
      SmartDashboard.putString("Order","Extending first");
      addCommands(
        new ExtendLevel1(m_ArmExtension).withTimeout(1), new RotateLevel1(m_ArmShoulder)
      );
    }
  }
}
