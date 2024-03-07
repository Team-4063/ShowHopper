// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final TalonFX shooterTopMotor = new TalonFX(Constants.MotorConstants.m_shooterTopMotor, Constants.MotorConstants.m_CANbus); //i added static but i think thats bad...ill come back later
  private final TalonFX shooterBottomMotor = new TalonFX(Constants.MotorConstants.m_shooterBottomMotor, Constants.MotorConstants.m_CANbus);

  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooterSub(double speed){
    shooterTopMotor.set(-speed);
    shooterBottomMotor.set(speed);
  }
  

  public void shooterAmp(){
    shooterTopMotor.set(Constants.SpeedConstants.kShootTopAmp);
    shooterBottomMotor.set(Constants.SpeedConstants.kShootBottomAmp);

  }



}
