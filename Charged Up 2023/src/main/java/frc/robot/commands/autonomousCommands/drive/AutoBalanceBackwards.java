// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.drive;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceBackwards extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private int state = 0;
  private int debounceCount = 0;

  private final BuiltInAccelerometer m_RioAccel;

  /** Creates a new DriveToDistanceNew. */
  public AutoBalanceBackwards(DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    m_DriveSubsystem.resetEncoders();
    m_RioAccel = m_DriveSubsystem.getRIOAccelerometer();
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.resetEncoders();
    state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = getAutoBalancingSpeed();
    m_DriveSubsystem.driveRaw(speed);
    SmartDashboard.putNumber("Autobal Speed", speed);
  }

// +Y --> -Z
// +Z --> -X
// +X --> -Y

  public double getPitch(){
    return Math.atan2((m_RioAccel.getY()),
    Math.sqrt(-m_RioAccel.getZ() * -m_RioAccel.getZ() + -m_RioAccel.getX() * -m_RioAccel.getX())) * 57.3;  }

  public double getRoll(){
    return Math.atan2(-m_RioAccel.getZ(), -m_RioAccel.getX()) * 57.3;
  }

  public void resetState(){
    state = 0;
  }

  public double getTilt(){
    double pitch = getPitch();
    double roll = getRoll();

    if(pitch + roll >= 0){
      return Math.sqrt((pitch * pitch) + (roll * roll));
    }else{
      return -1*Math.sqrt((pitch * pitch) + (roll * roll));
    }

  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  public double getAutoBalancingSpeed(){
    switch(state){
      case 0:
        SmartDashboard.putNumber("Autobalance state", state);
        SmartDashboard.putNumber("Tilt", getTilt());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());

        if(getTilt() < AutoBalanceConstants.onStationDegree){
          debounceCount++;
        }

        if(debounceCount > secondsToTicks(AutoBalanceConstants.debounceTime)){
          state = 1;
          debounceCount = 0;
          return AutoBalanceConstants.speedSlow;
        }
        return AutoBalanceConstants.speedFast;
      
      case 1:
        SmartDashboard.putNumber("Autobalance state", state);
        SmartDashboard.putNumber("Tilt", getTilt());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());

        if(getTilt() > AutoBalanceConstants.balancedDegree){
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(AutoBalanceConstants.debounceTime)){
          state = 2;
          debounceCount = 0;
          return 0.0;
        }
        return AutoBalanceConstants.speedSlow;

      case 2:
        SmartDashboard.putNumber("Autobalance state", state);
        SmartDashboard.putNumber("Tilt", getTilt());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());

        if(-1*Math.abs(getTilt()) >= AutoBalanceConstants.balancedDegree / 2){
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(AutoBalanceConstants.debounceTime)){
          state = 3;
          debounceCount = 0;
          return 0.0;
        }
        if(getTilt() <= AutoBalanceConstants.balancedDegree){
          return AutoBalanceConstants.speedExtraSlow*0.75;
        }else if (getTilt() >= -AutoBalanceConstants.balancedDegree){
          return -AutoBalanceConstants.speedExtraSlow*1.5;
        }

      case 3:
        SmartDashboard.putNumber("Autobalance state", state);
        SmartDashboard.putNumber("Tilt", getTilt());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());
        if(-1*Math.abs(getTilt()) <= AutoBalanceConstants.balancedDegree / 2){
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(AutoBalanceConstants.debounceTime)){
          state = 2;
          debounceCount = 0;
          return 0.0;
        }

        return 0.0;
    }
    return 0.0;
  }

}