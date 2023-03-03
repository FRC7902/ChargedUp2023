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
    public final static int SoftwareCurrentLimit = 33;

    public final static int kSensorUnitsPerRotation = 4096;

    public final static double kLevel0Percentage = 0.01;
    public final static double kLevel1Percentage = 0.33;
    public final static double kLevel2Percentage = 0.67;
    public final static double kLevel3Percentage = 1.0;

    public final static double ArmExtensionFeedForward = 0.0;

    public final static double ArmExtensionPower = 0.35;

    public final static double extendedMaxSoftLimitInInches = 27;//final and tested
    public final static double extendedMinSoftLimitInInches = 1.5; //dummy variable

    public final static int EncoderCPR = 2048; 
    public static final int kEncoderA = 8;
    public static final int kEncoderB = 9;

    public static final double kPD = 1.432;
    public static final double kPC = kPD*Math.PI;
    public static final int ZeroPosLimitSwitchDIO = 7;


  }

  public final static class ArmShoulderConstants {

    // CAN ID's
    public final static int ArmShoulderLeaderCAN = 4; // all dummies
    public final static int ArmShoulderFollowerCAN = 12;// 0

    public final static double ArmShoulderRotatePower = 0.5; // testing, not final
    public final static double ArmShoulderStop = 0.0; // testing, not final
    public final static double ArmShoulderFeedForwardMin = 0.12; //final and tested
    public final static double ArmShoulderFeedForwardMax = 0.22; //final and tested
    public final static double ArmShoulderFeedForwardDifference = ArmShoulderFeedForwardMax - ArmShoulderFeedForwardMin;
    public final static double angleAdjustmentDegrees = 71.57;
    public final static double angleAdjustmentRadians = Units.degreesToRadians(angleAdjustmentDegrees);

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

    //NEED TO TEST THESE VARIABLES
    public final static double kLevel0Degrees = 0.0; //off
    public final static double kLevel1Degrees = 30.0; //low
    public final static double kLevel2Degrees = 60.0; //medium
    public final static double kLevel3Degrees = 95.0; //high

    public final static double kLevel0EncoderTicks = (kLevel0Degrees/360) * EncoderCPR;
    public final static double kLevel1EncoderTicks = (kLevel1Degrees/360) * EncoderCPR;
    public final static double kLevel2EncoderTicks = (kLevel2Degrees/360) * EncoderCPR;
    public final static double kLevel3EncoderTicks = (kLevel3Degrees/360) * EncoderCPR;
    public final static double ticksPerDegree = EncoderCPR/360;

  }

  public static class GainConstants {

    /**
     * Choose based on what direction you want to be positive,
     * this does not affect motor invert.
     */
    public static boolean kMotorInvert = false;
    public static Object IntakeConstants;

    public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
    public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
    public final static Gains kGains_Velocit = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
    public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);
    /*
     * Firmware currently supports slots [0, 3] and can be used for either PID Set
     */
    public final static int SLOT_0 = 0;
    public final static int SLOT_1 = 1;
    public final static int SLOT_2 = 2;
    public final static int SLOT_3 = 3;
    /* ---- Named slots, used to clarify code ---- */
    public final static int kSlot_Distanc = SLOT_0;
    public final static int kSlot_Turning = SLOT_1;
    public final static int kSlot_Velocit = SLOT_2;
    public final static int kSlot_MotProf = SLOT_3;

  }

  public static final class IntakeConstants {
    public final static int IntakeCAN = 8; 
    public final static double SuckCubeSpeed = 0.5; //These need testing
    public final static double ShootCubeSpeed = 0.8;
    public final static double SuckConeSpeed = -0.5;
    public final static double ShootConeSpeed = -0.8;


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
