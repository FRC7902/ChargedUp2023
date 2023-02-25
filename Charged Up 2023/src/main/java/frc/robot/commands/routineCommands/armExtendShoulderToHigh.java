// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.teleopCommands.armExtension.ArmExtend;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class armExtendShoulderToHigh extends ParallelCommandGroup {
  /** Creates a new armExtendToLow. */
  public armExtendShoulderToHigh(ArmShoulder armShoulder, WPI_TalonSRX armShoulderLeader, ArmExtension armExtend) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new armShoulderRotateToAngle(armShoulder, armShoulderLeader, Constants.armShoulderRoutineConstants.shoulderExtendToHighAngle), new armExtendToDistance(armExtend, Constants.ArmExtensionConstants.extendedMaxInInches));
  }
}
