package frc.robot.commands.teleopCommands.camera;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;

public class SwitchCamera extends CommandBase {

    private CameraSubsystem m_camera;

    /** Creates a new SlowDriveForward. */
    public SwitchCamera(CameraSubsystem camera) {
        m_camera = camera;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_camera.cameraType == 1) {
            m_camera.cameraType = 2;
        } else {
            m_camera.cameraType = 1;
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

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
