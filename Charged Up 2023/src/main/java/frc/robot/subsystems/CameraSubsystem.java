// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {

  private UsbCamera camera;
  private Mat mat;
  private CvSource outputStream;
  private CvSink cvSink;
  private Thread visionThread;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {

  camera = CameraServer.startAutomaticCapture(0);
  camera.setResolution(80, 80);

  cvSink = CameraServer.getVideo();

  outputStream = CameraServer.putVideo("Stream", 80, 80);

  mat = new Mat();

  visionThread = new Thread(
    () -> {
      while(!Thread.interrupted()) {
        if(cvSink.grabFrame(mat) == 0) {
          outputStream.notifyError(cvSink.getError());

          continue;
        }
        Imgproc.rectangle(mat, 
        new Point(280, 80), 
        new Point(360, 120), 
        new Scalar (0,0,255));

        outputStream.putFrame(mat);

      }
    }
  );

  visionThread.setDaemon(true);
  visionThread.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
