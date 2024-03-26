// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberStop extends Command {
  /** Creates a new ClimberStop. */
  private final ClimberSubsystem m_climber;
  public ClimberStop(ClimberSubsystem climberSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climberSub;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.climberIn(SpeedConstants.kClimbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberIn(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_climber.climberStatus()){
      return false;
    } else{
      return true;
    }
  }
}
