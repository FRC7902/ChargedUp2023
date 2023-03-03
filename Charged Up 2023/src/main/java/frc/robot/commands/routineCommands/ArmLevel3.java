// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.armExtension.ExtendLevel3;
import frc.robot.commands.teleopCommands.armshoulder.RotateLevel3;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLevel3 extends SequentialCommandGroup {
  /** Creates a new armExtendToLow. */
  public ArmLevel3(ArmShoulder armShoulder, ArmExtension armExtend) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RotateLevel3(armShoulder).andThen(new ExtendLevel3(armExtend)));
  }
}
