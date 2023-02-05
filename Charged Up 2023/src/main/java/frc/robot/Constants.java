// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class DriveConstants {

    public final static int DrivetrainRightLeaderCAN = 4;
    public final static int DrivetrainRightFollowerCAN = 3;

    public final static int DrivetrainLeftLeaderCAN = 11;
    public final static int DrivetrainLeftFollowerCAN = 12;
  }

  public final static class ArmConstants {
    public final static int ArmExtensionLeaderCAN = 10;
    public final static int ArmExtensionFollowerCAN = 9;

    // dummies
    public final static int ArmShoulderLeaderCAN = 4;
    public final static int ArmShoulderFollowerCAN = 3;//0
    public final static double ArmShoulderRotateIn = 0.25; // testing, not final
    public final static double ArmShoulderRotateOut = -0.25; // testing, not final
    public final static double ArmShoulderStop = 0.0; // testing, not final

    public final static int IntakeCAN = 0;

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

  public static final class IOConstants {
    public static final int kDriverStick = 0;
    public static final int kOperatorStick = 1;

    // Joystick Buttons
    public static final int kA = 1,
        kB = 2,
        kX = 3,
        kY = 4,
        kLB = 5,
        kRB = 6,
        kMENU = 7,
        kSTART = 8,
        kLA = 9,
        kRA = 10;

    // Joystick Axis
    public static final int kLX = 0,
        kLY = 1,
        kLT = 2,
        kRT = 3,
        kRX = 4,
        kRY = 5,
        kDX = 6,
        kDY = 7;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
