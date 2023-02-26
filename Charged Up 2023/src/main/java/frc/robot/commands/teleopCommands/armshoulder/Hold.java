// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.armshoulder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmShoulder;

public class Hold extends CommandBase {

  //private ArmShoulder m_armShoulder;
  private ArmShoulder m_armShoulder;
  private ArmExtension m_ArmExtension;
  private WPI_TalonSRX m_armMotor;
  private double m_minFeedForward = Constants.ArmShoulderConstants.ArmShoulderFeedForwardMin;
  private double m_maxFeedForward = Constants.ArmShoulderConstants.ArmShoulderFeedForwardMax;
  int count = 0;

  /** Creates a new RotateOut. */
  public Hold(ArmShoulder armShoulder, WPI_TalonSRX armShoulderLeader, ArmExtension armExtension) { 
    m_armShoulder = armShoulder;
    m_armMotor = armShoulderLeader;
    m_ArmExtension = armExtension;
    //addRequirements(armShoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armShoulder.stopMotor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Arm holding..");

    //absolute position gets the location of the arm in ticks (4096 per revolution)
    int absolutePosition = m_armMotor.getSensorCollection().getQuadraturePosition();

    //convert from ticks to degrees
    double deg = (double)absolutePosition/4096 * 360;

    deg -= Constants.ArmShoulderConstants.restDegreesFromHorizontal;

    double power = (m_minFeedForward + ((m_maxFeedForward - m_minFeedForward) * m_ArmExtension.percentExtension) * Math.cos(deg));
    //m_armShoulder.setPower(power);
    m_armShoulder.setPower(0.1);

    count++;

    if(count >= 10){
      System.out.println("HOLDING POS: " + deg + " " + absolutePosition);
      count = 0;
    }

    // System.out.println("POS: " + deg + " " + absolutePosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
