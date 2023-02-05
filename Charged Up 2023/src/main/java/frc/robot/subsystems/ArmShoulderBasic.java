// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.sim.PhysicsSim;

public class ArmShoulderBasic{
    //extends TrapezoidProfileSubsystem
    private final WPI_TalonSRX armShoulderLeader = new WPI_TalonSRX(ArmConstants.ArmShoulderLeaderCAN);
    private final WPI_VictorSPX armShoulderFollower = new WPI_VictorSPX(ArmConstants.ArmShoulderFollowerCAN);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.ArmSVolts, ArmConstants.ArmGVolts,
    ArmConstants.ArmVVoltSecondPerRad, ArmConstants.ArmAVoltSecondSquaredPerRad);
    private final Joystick m_joystick = new Joystick(Constants.IOConstants.kOperatorStick);
	private StringBuilder _sb = new StringBuilder();
    private boolean _lastButton1 = false;
    private double targetPositionRotations;
    private final XboxController m_driverController = new XboxController(Constants.IOConstants.kOperatorStick);

  // Need encoder
  // Need limit switch

  /** Creates a new ArmSubsystem. */
  public ArmShoulderBasic() {


    //TRAPEZOID

    // super(new TrapezoidProfile.Constraints(ArmConstants.MaxVelocityRadPerSecond,
    //     ArmConstants.MaxAccelerationRadPerSecSquared), ArmConstants.ArmOffsetRads);
    // armShoulderFollower.follow(armShoulderLeader);
    // armShoulderLeader.setInverted(false);
    // armShoulderFollower.setInverted(InvertType.FollowMaster);
    // armShoulderLeader.setPID(ArmConstants.ArmPosition, 0, 0);

    armShoulderFollower.follow(armShoulderLeader);
    armShoulderLeader.setInverted(false);

  }

  public void setPower(double power) {
    armShoulderLeader.set(power);

    // if statements needed for testing
  }

  public void stopMotor() {
    armShoulderLeader.stopMotor();
  }

  //TRAPEZIOD
  // @Override
  // protected void useState(TrapezoidProfile.State state) {
  //   double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  //   // Add the feedforward to the PID output to get the motor output
  //   armShoulderLeader.setSetpoint(WPI_TalonSRX.PIDMode.ArmPosition, setpoint.position, feedforward / x);
  // }

  void commonLoop() {
    /* Gamepad processing */
    double leftYstick = m_driverController.getRawAxis(Constants.IOConstants.kOperatorStick); //Left stick
    boolean button1 = m_driverController.getRawButton(Constants.IOConstants.kA);	// A-Button
    boolean button2 = m_driverController.getRawButton(Constants.IOConstants.kB);	// B-Button

    /* Get Talon/Victor's current output percentage */
    double motorOutput = armShoulderLeader.getMotorOutputPercent();

    /* Deadband gamepad */
    if (Math.abs(leftYstick) < 0.10) {
        /* Within 10% of zero */
        leftYstick = 0;
    }

    /* Prepare line to print */
    _sb.append("\tout:");
    /* Cast to int to remove decimal places */
    _sb.append((int) (motorOutput * 100));
    _sb.append("%");	// Percent

    _sb.append("\tpos:");
    _sb.append(armShoulderLeader.getSelectedSensorPosition(0));
    _sb.append("u"); 	// Native units

    /**
     * When button 1 is pressed, perform Position Closed Loop to selected position,
     * indicated by Joystick position x10, [-10, 10] rotations
     */
    if (button1) {
        /* Position Closed Loop */

        /* 10 Rotations * 4096 u/rev in either direction */
        targetPositionRotations = (leftYstick + 0.1) * 10.0 * 4096;
        armShoulderLeader.set(ControlMode.Position, targetPositionRotations);
    }

    /* When button 2 is held, just straight drive */
    if (button2) {
        /* Percent Output */

        armShoulderLeader.set(ControlMode.PercentOutput, leftYstick);
    }

    /* If Talon is in position closed-loop, print some more info */
    if (armShoulderLeader.getControlMode() == ControlMode.Position) {
        /* ppend more signals to print when in speed mode. */
        _sb.append("\terr:");
        _sb.append(armShoulderLeader.getClosedLoopError(0));
        _sb.append("u");	// Native Units

        _sb.append("\ttrg:");
        //_sb.append(targetPositionRotations);
        _sb.append("u");	/// Native Units
    }

    /**
     * Print every ten loops, printing too much too fast is generally bad
     * for performance.
     */
    // if (++_loops >= 10) {
    //     _loops = 0;
    //     System.out.println(_sb.toString());
    // }

    /* Reset built string for next loop */
    _sb.setLength(0);
    
    /* Save button state for on press detect */
    _lastButton1 = button1;
}

/**
 * This function is called periodically during operator control
 */
public void teleopPeriodic() {
    commonLoop();
}

public void setLocation(ControlMode mode, double location){
    armShoulderLeader.set(mode, location);
}
}
