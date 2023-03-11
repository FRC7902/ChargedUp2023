// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmShoulderConstants;

import frc.robot.FireBirdsUtils;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmShoulder extends SubsystemBase {
  private static ArmExtension m_ArmExtension;

  // Declare motor controllers
  private final static WPI_TalonSRX armShoulderLeader = new WPI_TalonSRX(ArmShoulderConstants.ArmShoulderLeaderCAN);
  private final static WPI_VictorSPX armShoulderFollower = new WPI_VictorSPX(
  ArmShoulderConstants.ArmShoulderFollowerCAN);

  private final static FireBirdsUtils util = new FireBirdsUtils();

  /** Target angle **/
  private static double targetPosition;

  /** Object of a simulated arm **/
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getCIM(2), 139.78, 6.05, 1, Units.degreesToRadians(-ArmShoulderConstants.restDegreesFromHorizontal),6,true);

  // Simulation of TalonSRX
  private final TalonSRXSimCollection armShoulderLeaderSim = armShoulderLeader.getSimCollection();

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(80, 80); // the overall arm
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30); // pivot point
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm = m_armPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(armSim.getAngleRads()),
          6,
          new Color8Bit(Color.kAliceBlue)));

  // Need limit switch

  /** Creates a new ArmSubsystem. */
  public ArmShoulder(ArmExtension armExtension) {
    m_ArmExtension = armExtension;
    armShoulderLeader.configFactoryDefault();
    armShoulderFollower.configFactoryDefault();

    armShoulderFollower.follow(armShoulderLeader);
    armShoulderLeader.setInverted(false);
    armShoulderFollower.setInverted(InvertType.FollowMaster);
    armShoulderLeader.configVoltageCompSaturation(12,0);

    armShoulderLeader.config_kP(0, 1.5);

    armShoulderLeader.configMotionCruiseVelocity(1000);
    armShoulderLeader.configMotionAcceleration(4000);

    // armShoulderLeader.config_kP(Constants.GainConstants.kSlot_Distanc,
    // Constants.GainConstants.kGains_Distanc.kP,
    // Constants.ArmShoulderConstants.kTimeoutMs);
    // armShoulderLeader.configMotionAcceleration(2000,
    // Constants.ArmShoulderConstants.kTimeoutMs);
    // armShoulderLeader.configMotionCruiseVelocity(2000,
    // Constants.ArmShoulderConstants.kTimeoutMs);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // Encoder
    armShoulderLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        0);

    if (RobotBase.isSimulation()) {
      armShoulderLeader.setInverted(false);
      armShoulderLeader.setSensorPhase(false);
    } else {
      armShoulderLeader.setInverted(false);
      armShoulderLeader.setSensorPhase(true);
    }

    // limit switch
    armShoulderLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);

    armShoulderLeader.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
    targetPosition = getPosition();

  }

  public void setPower(double power) {
    armShoulderLeader.set(power);

  }

  public void setPosition(double demand0, double demand1) {
    armShoulderLeader.set(ControlMode.Position, demand0, DemandType.ArbitraryFeedForward, 0);
  }

  public void setTargetPosition(double newTargetPosition) {
    targetPosition = newTargetPosition;
  }

  public void set(ControlMode mode, double value) {
    armShoulderLeader.set(mode, value);
  }

  // public double getFollowerPower() {
  // return armShoulderFollower.get();
  // }

  public double getLeaderPower() {
    return armShoulderLeader.get();
  }

  public boolean atZeroPos() {
    return armShoulderLeader.isRevLimitSwitchClosed() == 0; // switch is open
  }

  public void stopMotor() {
    armShoulderLeader.stopMotor();
  }

  public double getPosition() {
    // absolute position gets the location of the arm in ticks (4096 per revolution)
    return armShoulderLeader.getSelectedSensorPosition();
  }

  // METHOD - GET NEW POSITION

  /**
   * This function is called periodically during operator control
   */
  int counter = 0;

  @Override
  public void simulationPeriodic() {

    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(armShoulderLeaderSim.getMotorOutputLeadVoltage());
    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    armShoulderLeaderSim
        .setQuadratureRawPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), ArmShoulderConstants.EncoderCPR));

    m_arm.setAngle(Math.toDegrees(armSim.getAngleRads()));

    // Zero the limit switch in simulation
    if (armSim.getAngleRads() == Units.degreesToRadians(-ArmShoulderConstants.restDegreesFromHorizontal)) {

      armShoulderLeader.getSensorCollection().setQuadraturePosition(0, 0);
    }

    armShoulderLeaderSim.setAnalogPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), 4096));

  }

  @Override
  public void periodic() {

    if (armShoulderLeader.isRevLimitSwitchClosed() == 1) {
      armShoulderLeader.setSelectedSensorPosition(0);

    }

    double currentPosition = getPosition(); //in raw sensor units

    double adjusted_feedForward = (ArmShoulderConstants.ArmShoulderFeedForwardMin
        + (ArmShoulderConstants.ArmShoulderFeedForwardDifference * m_ArmExtension.getPercentExtension()))
        * Math.cos(util.CTRESensorUnitsToRads(targetPosition, ArmShoulderConstants.EncoderCPR)-
            ArmShoulderConstants.angleAdjustmentRadians);

    SmartDashboard.putNumber("Current Arm Position: ", currentPosition);
    SmartDashboard.putNumber("Target Position", targetPosition);
    SmartDashboard.putNumber("Leader Voltage", armShoulderLeader.getMotorOutputVoltage());
    SmartDashboard.putNumber("Adjusted feedforward", adjusted_feedForward);
    SmartDashboard.putNumber("Shoulder Current (A)", armShoulderLeader.getSupplyCurrent());
    SmartDashboard.putNumber("Shoulder Error", armShoulderLeader.getClosedLoopError(0));

    if (RobotBase.isSimulation()) {
      armShoulderLeader.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward,
          adjusted_feedForward);

    } else {
      armShoulderLeader.set(ControlMode.MotionMagic, targetPosition*2, DemandType.ArbitraryFeedForward,
          adjusted_feedForward);
    }

    // Position control to current position variable

  }

}
