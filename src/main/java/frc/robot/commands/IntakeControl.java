// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class IntakeControl extends Command {
  /** Creates a new IntakeControl. */
  private final IntakeSubsystem m_intake;
  private final TransitionSubsystem m_transition;

  public IntakeControl(IntakeSubsystem intakeSub, TransitionSubsystem transitionSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intakeSub;
    m_transition = transitionSub;

    addRequirements(intakeSub, transitionSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeRun(SpeedConstants.kIntakeSpeed);
    m_transition.transitionRun(SpeedConstants.kTransitionSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.intakeRun(0);
    m_transition.transitionRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if(m_transition.notePosition() > SensorConstants.kNotePosition){
      return true;
    }else{
    return false;} */
    return m_transition.noteIn();
  }
}
