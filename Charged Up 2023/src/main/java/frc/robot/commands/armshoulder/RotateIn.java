package frc.robot.commands.armshoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmShoulderBasic;

public class RotateIn extends CommandBase {
  // private ArmShoulder m_armShoulder;
  private ArmShoulderBasic m_armShoulder;

  /** Creates a new RotateIn. */
  public RotateIn(ArmShoulderBasic armShoulder) {
    m_armShoulder = armShoulder;
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
    System.out.println("Arm rotating in..");
    m_armShoulder.setPower(ArmConstants.ArmShoulderRotateIn);

    //m_armShoulder.setLocation(ControlMode.Position, 10.0*4096);

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
