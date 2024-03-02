// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final TalonFX pivotMotor = new TalonFX(Constants.MotorConstants.m_pivotMotor, Constants.MotorConstants.m_CANbus);

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  public PivotSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean connected = pivotEncoder.isConnected();

    //int frequency = pivotEncoder.getFrequency();

    double output = pivotEncoder.get();

    double angle = output * 360;

    SmartDashboard.putBoolean("Connected", connected);
    //SmartDashboard.putNumber("Frequency", frequency);
    //SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Angle", angle);

  }

  public void pivotRun(double speed){
    pivotMotor.set(speed);
  }

  public double pivotPosition(){
    return pivotEncoder.get();
  }

}
