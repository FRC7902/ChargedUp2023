package frc.robot.commands.armshoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmShoulder;

public class RotateIn extends CommandBase {
  private ArmShoulder m_armShoulder;

  /** Creates a new RotateIn. */
  public RotateIn(ArmShoulder armShoulder) {
    m_armShoulder = armShoulder;
    addRequirements(armShoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armShoulder.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armShoulder.setPower(ArmConstants.ArmShoulderRotateIn);
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
