// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.FinalAutonCubeOnHigh;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autonomousCommands.drive.AutoBalance;
import frc.robot.commands.autonomousCommands.drive.DriveToDistance;
import frc.robot.commands.autonomousCommands.drive.AutoBalanceSetPower;
import frc.robot.commands.teleopCommands.armExtension.*;
import frc.robot.commands.teleopCommands.armshoulder.*;
import frc.robot.commands.teleopCommands.intake.*;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleStart extends SequentialCommandGroup {
  /** Creates a new PlaceCubeOnHigh. */
  public MiddleStart(ArmShoulder armShoulder, ArmExtension armExtend, IntakeSubsystem intake, DriveSubsystem drive, AutoBalance autoBalance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateLevel2(armShoulder).withTimeout(Constants.ArmShoulderConstants.ShoulderBufferTimeInSeconds),
      new ExtendLevel3(armExtend).withTimeout(Constants.ArmExtensionConstants.ExtensionBufferTimeInSeconds),
      new shootCube(intake).withTimeout(0.2),
      new IntakeStop(intake).withTimeout(0.1),
      new ExtendLevel0(armExtend).withTimeout(Constants.ArmExtensionConstants.ExtensionBufferTimeInSeconds),
      new RotateLevel0(armShoulder).withTimeout(Constants.ArmShoulderConstants.ShoulderBufferTimeInSeconds),
      new AutoBalanceSetPower(drive, autoBalance)
    );
  }
}
