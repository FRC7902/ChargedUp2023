// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

    public final static int DrivetrainRightLeaderCAN = 13; 
    public final static int DrivetrainRightFollowerCAN = 14;
    public final static int DrivetrainLeftLeaderCAN = 15;
    public final static int DrivetrainLeftFollowerCAN = 16;

    public final static int SoftwareCurrentLimit = 50;
    public final static double OutputGearRatio = 1.0 / 10.71;
    public final static double WheelCircumferenceInInches = 6 * Math.PI;
    public final static double DistancePerWheelRotationFeet = WheelCircumferenceInInches * 12;

    public final static double DistanceBetweenWheels = 20; // inches

    public final static double SlowDriveSpeed = 0.1;

    // need deadband
  }

  public final static class armShoulderRoutineConstants{
    public final static double shoulderExtendToHighAngle = 100;//dummy
    public final static double shoulderRetractToLowAngle = 0;//dummy
  }

  public final static class ArmExtensionConstants {
    // CAN ID's
    public final static int ArmExtensionLeaderCAN = 11;
    public final static int ArmExtensionFollowerCAN = 7;
    public final static int SoftwareCurrentLimit = 20;

    public final static int kSensorUnitsPerRotation = 4096;

    public final static double ArmExtensionFeedForward = 0.0;

    public final static double ArmExtensionPower = 0.35;

    public final static double extendedMaxSoftLimitInInches = 27;//final and tested
    public final static double extendedMinSoftLimitInInches = 0.5;
    public final static double extendedLevel1SoftLimitInInches = 0.41836*extendedMaxSoftLimitInInches;
    public final static double extendedLevel2SoftLimitInInches = 0.66*extendedMaxSoftLimitInInches;
    
    public final static double kLevel0Percentage = 0.01;
    public final static double kLevel1Percentage = 0.41836; //ground engagement
    public final static double kLevel2Percentage = (11/extendedMaxSoftLimitInInches); //mid engagement
    public final static double kLevel3Percentage = 1.0; //high shooting
    public final static int EncoderCPR = 2048; 
    public static final int kEncoderA = 8;
    public static final int kEncoderB = 9;

    public static final double kPD = 1.432;
    public static final double kPC = kPD*Math.PI;
    public static final int ZeroPosLimitSwitchDIO = 7;

    public static final double extensionHomingPower = 0.5;

    public static final double ExtensionBufferTimeInSeconds = 0.75;

  }

  public final static class ArmShoulderConstants {

    // CAN ID's
    public final static int ArmShoulderLeaderCAN = 4; // all dummies
    public final static int ArmShoulderFollowerCAN = 12;// 0

    public final static double ArmShoulderRotatePower = 0.5; // testing, not final
    public final static double ArmShoulderStop = 0.0; // testing, not final
    public final static double ArmShoulderFeedForwardMin = 0.26; //final and tested
    public final static double ArmShoulderFeedForwardMax = 0.5; //final and tested
    public final static double ArmShoulderFeedForwardDifference = ArmShoulderFeedForwardMax - ArmShoulderFeedForwardMin;
    public final static double angleAdjustmentDegrees = 71.57;
    public final static double angleAdjustmentRadians = Units.degreesToRadians(angleAdjustmentDegrees);

    public static final double ShoulderBufferTimeInSeconds = 0.75;


    public final static double EncoderToOutputRatio = 0.5;
    public final static double restDegreesFromHorizontal = 75;


    /** SRX Mag Encoder counts per revolution**/
    public final static int EncoderCPR = 4096;

    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from
     * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
     * configuration.
     */
    public static final int kSlotIdx = 0;

    /**
     * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /**
     * Set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;

    /* Choose so that Talon does not report sensor out of phase */
    public static boolean kSensorPhase = true;

    //All these values are measured off the pillar, not the vertical
    public final static double kLevel0Degrees = 2.0; //off
    public final static double kLevel1Degrees = 15; //ground
    public final static double kLevel2Degrees = 60.0; //Mid-height/grabbing from human player
    public final static double kLevel3Degrees = 75.0; //high

    public final static double kLevel0EncoderTicks = (kLevel0Degrees/360) * EncoderCPR;
    public final static double kLevel1EncoderTicks = (kLevel1Degrees/360) * EncoderCPR;
    public final static double kLevel2EncoderTicks = (kLevel2Degrees/360) * EncoderCPR;
    public final static double kLevel3EncoderTicks = (kLevel3Degrees/360) * EncoderCPR;
    public final static double ticksPerDegree = EncoderCPR/360;

    public static final double shoulderHomingPower = 0.5;
  }

  // public static final CameraConstants {
  //   public static final int resX = 80;
  //   public static final int resY = 80;



  // }
  
  

  public static final class IntakeConstants {
    public final static int IntakeCAN = 8; 
    public final static double SuckCubeSpeed = 0.8; //These need testing
    public final static double ShootCubeSpeed = -0.8;
    public final static double SuckConeSpeed = -0.8;
    public final static double ShootConeSpeed = 0.8;


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

}
