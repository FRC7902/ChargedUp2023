// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands.drive;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  double PitchAngle;
  double RollAngle;
  double TiltAngle;
  int state = 0;
  int debounceCount = 0;

  PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.PigeonCAN);

  /** Creates a new DriveToDistanceNew. */
  public AutoBalance(DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    m_DriveSubsystem.resetEncoders();
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PitchAngle = m_pigeon.getPitch();
    RollAngle = m_pigeon.getRoll();

  }

  public double getPitch(){
    return m_pigeon.getPitch();
  }

  public double getRoll(){
    return m_pigeon.getPitch();
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

  public double AutoBalancingSpeed(){
    switch(state){

      case 0:
        if(getTilt() > AutoBalanceConstants.onStationDegree){
          debounceCount++;
        }

        if(debounceCount > secondsToTicks(AutoBalanceConstants.debounceTime)){
          state = 1;
          debounceCount = 0;
          return AutoBalanceConstants.speedSlow;
        }
        return AutoBalanceConstants.speedFast;
      
      case 1:
        if(getTilt() < AutoBalanceConstants.balancedDegree){
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(AutoBalanceConstants.debounceTime)){
          state = 2;
          debounceCount = 0;
          return 0.0;
        }
        return AutoBalanceConstants.speedSlow;

      case 2:
        if(Math.abs(getTilt()) <= AutoBalanceConstants.balancedDegree / 2){
          debounceCount++;
        }
        if(debounceCount > secondsToTicks(AutoBalanceConstants.debounceTime)){
          state = 3;
          debounceCount = 0;
          return 0.0;
        }
        if(getTilt() >= AutoBalanceConstants.balancedDegree){
          return AutoBalanceConstants.speedExtraSlow;
        }else if (getTilt() <= -1*AutoBalanceConstants.balancedDegree){
          return -1*AutoBalanceConstants.speedExtraSlow;
        }

      default:
        return 0.0;
    }
  }

}