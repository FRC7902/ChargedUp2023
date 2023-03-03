// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.routineCommands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
  public ArmLevel1(ArmShoulder armShoulder, ArmExtension armExtend) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double currentArmAngle = armShoulder.getPosition();

    if(currentArmAngle < Constants.ArmShoulderConstants.kLevel1EncoderTicks){
        new RotateLevel1(armShoulder).andThen(new ExtendLevel1(armExtend));
    }else{
        new ExtendLevel1(armExtend).andThen(new RotateLevel1(armShoulder));
    }
  }
}
