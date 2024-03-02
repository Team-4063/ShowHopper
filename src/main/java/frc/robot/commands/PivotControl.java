// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotControl extends Command {
  /** Creates a new PivotControl. */
  private final PivotSubsystem m_pivot;
  private final double m_position;

  public PivotControl(double position,PivotSubsystem pivotSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivotSub;
    m_position = position;
    addRequirements(m_pivot);
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.pivotRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
