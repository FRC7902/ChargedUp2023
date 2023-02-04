// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class DriveConstants{

        public final static int DrivetrainRightLeaderCAN = 4;
        public final static int DrivetrainRightFollowerCAN = 3;

        public final static int DrivetrainLeftLeaderCAN = 11;
        public final static int DrivetrainLeftFollowerCAN = 12;
    }

    public final static class ArmConstants{ 
      public final static int ArmLeaderCAN = 10;
      public final static int ArmFollowerCAN = 9;

    
      // dummy variables 
      public final static int ArmSVolts = 0;
      public final static int ArmGVolts = 0;
      public final static int ArmVVoltSecondPerRad = 0;
      public final static int ArmAVoltSecondSquaredPerRad = 0;

      public final static int MaxVelocityRadPerSecond = 0;
      public final static int MaxAccelerationRadPerSecSquared = 0;
      public final static int ArmOffsetRads = 0;

      public final static int ArmPosition = 0;

    }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
