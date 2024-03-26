// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SensorConstants;

public class TransitionSubsystem extends SubsystemBase {
  /** Creates a new TransitionSubsystem. */
  private final WPI_VictorSPX transitionMotor = new WPI_VictorSPX(MotorConstants.m_transitionMotor);

  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  //private final AnalogInput m_ultrasonic = new AnalogInput(3);
  private final DigitalInput m_beamSensor = new DigitalInput(SensorConstants.beamReflected);

  public TransitionSubsystem() {}
//  Vcc/512
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Color detectedColor = m_colorSensor.getColor();

    //double IR = m_colorSensor.getIR();

    //SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("IR", IR);

    //int proximity = m_colorSensor.getProximity();
    //double proximity = m_ultrasonic.getVoltage(); //
    


    //SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putBoolean("Reflected", m_beamSensor.get());
  
  }

  public void transitionRun(double speed){
    transitionMotor.set(speed);
  }
/*
  public double notePosition(){
    return m_colorSensor.getProximity();
   //return m_ultrasonic.getVoltage();
  }
*/
  public boolean noteIn(){
    return m_beamSensor.get();
  }
}
