// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.armExtension.*;
import frc.robot.commands.teleopCommands.armshoulder.*;
import frc.robot.commands.teleopCommands.intake.*;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.IntakeSubsystem;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCubeOnHigh extends SequentialCommandGroup {
  /** Creates a new PlaceCubeOnHigh. */
  public PlaceCubeOnHigh(ArmShoulder armShoulder, ArmExtension armExtend, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateLevel3(armShoulder),
      new ExtendLevel3(armExtend),
      new shootCube(intake).withTimeout(1),
      new IntakeStop(intake),
      new ExtendLevel0(armExtend),
      new RotateLevel0(armShoulder)
    );
  }
}
