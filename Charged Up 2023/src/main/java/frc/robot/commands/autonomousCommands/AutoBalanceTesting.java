// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomousCommands.drive.AutoBalanceBackwards;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceTesting extends SequentialCommandGroup {
  /** Creates a new PlaceCubeOnHigh. */
  public AutoBalanceTesting(DriveSubsystem driveSubsystem, AutoBalanceBackwards autoBalance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    autoBalance.resetState();
    addCommands(
        new AutoBalanceBackwards(driveSubsystem)
    );
  }
}
