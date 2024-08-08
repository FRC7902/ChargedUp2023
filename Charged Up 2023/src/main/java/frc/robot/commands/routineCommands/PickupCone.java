// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.armshoulder.RotateLevel2;
import frc.robot.commands.teleopCommands.armshoulder.RotateLevel3;
import frc.robot.commands.teleopCommands.intake.suckCone;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupCone extends ParallelCommandGroup {

  private final ArmShoulder m_ArmShoulder;
  private final IntakeSubsystem m_Intake;
  /** Creates a new armExtendToLow. */
  public PickupCone(ArmShoulder armShoulder, ArmExtension armExtend, IntakeSubsystem intake) {
    m_ArmShoulder = armShoulder;
    m_Intake = intake;
    addCommands(
        new suckCone(m_Intake),
        new SequentialCommandGroup(
          new RotateLevel2(m_ArmShoulder).withTimeout(1),
          new RotateLevel3(m_ArmShoulder)
          )
    );

  }
}
