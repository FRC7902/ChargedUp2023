// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToDistance extends PIDCommand {
  /** Creates a new DriveToDistance. */
  public DriveToDistance(double targetDistanceInFeet, DriveSubsystem m_driveSubsytem) {
    super(
        // The controller that the command will use
        new PIDController(0.25, 0.1, 0.15), //idk what to set this to, these are just dummies
        // This should return the measurement
        () -> m_driveSubsytem.getLeftEncoder().getPosition(), //in inches
        // This should return the setpoint (can also be a constant)
        () -> targetDistanceInFeet*12, //in inches
        // This uses the output
        output -> {
          double adjustment = output/(targetDistanceInFeet*12);
          m_driveSubsytem.driveToDistance(adjustment, adjustment);
        }, m_driveSubsytem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // @Override
  // public void end(boolean interrupted){
  //   m_driveSubsystem.resetEncoders();
  // }
}
