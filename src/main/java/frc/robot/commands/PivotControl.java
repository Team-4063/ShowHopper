// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PivotControl extends Command {
  /** Creates a new PivotControl. */
  private final PivotSubsystem m_pivot;
  private final ShooterSubsystem m_shooter;
  private final double m_position;
  private final double m_shooterSpeed;
  private final double m_shooterSpeed2;

  public PivotControl(double position, PivotSubsystem pivotSub, ShooterSubsystem shooterSub, double shooterSpeed, double shooterSpeed2) { //took this out - double shooterSpeed, double shooterSpeed2
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivotSub;
    m_position = position;
    m_shooter = shooterSub;
    m_shooterSpeed = shooterSpeed;
    m_shooterSpeed2 = shooterSpeed2;
    addRequirements(m_pivot);
    addRequirements(m_shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_pivot.pivotPosition() > m_position){
    m_pivot.pivotRun(SpeedConstants.kPivotSpeed);
   
    } else{
      m_pivot.pivotRun(-SpeedConstants.kPivotSpeed);
    }
  
      //m_shooter.shooterSub(m_shooterSpeed, m_shooterSpeed2);
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.pivotRun(0);
   // m_shooter.shooterSub(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
