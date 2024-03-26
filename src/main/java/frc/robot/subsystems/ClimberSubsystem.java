// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SensorConstants;


public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final TalonFX climberMotor = new TalonFX(MotorConstants.m_climberMotor, MotorConstants.m_CANbus);

  private final DigitalInput climberSwitch = new DigitalInput(SensorConstants.climberLimit);

  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber In", climberStatus());
  }

  public void climberOut(double speed){
    climberMotor.set(-speed);
  }

  public void climberIn(double speed){
    climberMotor.set(speed);
  }

  public boolean climberStatus(){
    return climberSwitch.get();
  }

  
}
