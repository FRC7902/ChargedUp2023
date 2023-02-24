package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class DirectionBOuttake extends CommandBase {

    private Intake m_intake;
  
    /** Creates a new RotateOut. */
    public DirectionBOut(Intake intake) { 
      m_intake = intake;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_intake.stopMotor();
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_intake.setPower(IntakeConstants.DirectionBSpeed);
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
