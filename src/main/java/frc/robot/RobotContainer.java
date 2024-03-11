// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.ShooterControl;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {

  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  private final TransitionSubsystem TransitionSubsystem = new TransitionSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.m_driverControllerPort); // driver
  private final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.m_operatorControllerPort);//operator
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /*Path follower*/
  private Command runAuto = drivetrain.getAutoPath("testestAuto");

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    m_driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
       // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // --------------reset the field-centric heading on left bumper press-------------------------
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
   

//    m_driverController.a().onTrue(new RunCommand(() -> IntakeSubsystem.intakeRun(Constants.SpeedConstants.kIntakeSpeed), IntakeSubsystem))
//                .onFalse(Commands.runOnce(() -> IntakeSubsystem.intakeRun(0), IntakeSubsystem));
    
    //-------------------Transition Shoot-----------------
    m_driverController.rightTrigger().onTrue(new RunCommand(() -> TransitionSubsystem.transitionRun(Constants.SpeedConstants.kTransitionSpeed), TransitionSubsystem))
              .onFalse(Commands.runOnce(() -> TransitionSubsystem.transitionRun(0), TransitionSubsystem));
    //----------------Transition Reverse------------------
    m_driverController.leftTrigger().onTrue(new RunCommand(() -> TransitionSubsystem.transitionRun(-Constants.SpeedConstants.kTransitionSpeed), TransitionSubsystem))
              .onFalse(Commands.runOnce(() -> TransitionSubsystem.transitionRun(0), TransitionSubsystem));



    //-------------------Shooter Buttons also pivot------------------
   // m_operatorController.x().onTrue(new PivotControl(Constants.SensorConstants.kWoofPosition, pivotSubsystem, shooterSubsystem, Constants.SpeedConstants.kShootSubwoofer, Constants.SpeedConstants.kShootSubwoofer ))
                       //     .onFalse(Commands.runOnce(() -> pivotSubsystem.pivotRun(0), pivotSubsystem));
    
    //shooter sub
    m_operatorController.x().onTrue(new RunCommand(() -> shooterSubsystem.shooterSub(Constants.SpeedConstants.kShootSubwoofer), shooterSubsystem))
                .onFalse(Commands.runOnce(() -> shooterSubsystem.shooterSub(SpeedConstants.kNoShoot), shooterSubsystem));
   // m_operatorController.y().onTrue(new PivotControl(Constants.SensorConstants.kPodiumPosition, pivotSubsystem, shooterSubsystem, Constants.SpeedConstants.kShootPodium, Constants.SpeedConstants.kShootPodium))
                           // .onFalse(Commands.runOnce(() -> pivotSubsystem.pivotRun(0), pivotSubsystem));
    //shooter podium
    m_operatorController.y().onTrue(new RunCommand(() -> shooterSubsystem.shooterSub(Constants.SpeedConstants.kShootPodium), shooterSubsystem))
                .onFalse(Commands.runOnce(() -> shooterSubsystem.shooterSub(SpeedConstants.kNoShoot), shooterSubsystem));
    // m_operatorController.b().onTrue(new PivotControl(Constants.SensorConstants.kAmpPosition, pivotSubsystem, shooterSubsystem, Constants.SpeedConstants.kShootTopAmp, Constants.SpeedConstants.kShootBottomAmp))
                            //.onFalse(Commands.runOnce(() -> pivotSubsystem.pivotRun(0), pivotSubsystem));
    //shooter amp
     m_operatorController.b().onTrue(new RunCommand(() -> shooterSubsystem.shooterAmp(),shooterSubsystem))
               .onFalse(Commands.runOnce(() -> shooterSubsystem.shooterSub(SpeedConstants.kNoShoot), shooterSubsystem));
    //stop shooter
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> shooterSubsystem.shooterSub(0.0), shooterSubsystem));

    //shoot launch
    m_operatorController.rightBumper().onTrue(new RunCommand(() -> shooterSubsystem.shooterSub(Constants.SpeedConstants.kShootLaunch), shooterSubsystem))
                .onFalse(Commands.runOnce(() -> shooterSubsystem.shooterSub(SpeedConstants.kNoShoot), shooterSubsystem));
    



    //-----------------------------------pivot fine-tuning------------------------------------tobecont
    pivotSubsystem.setDefaultCommand(new RunCommand(() -> pivotSubsystem.pivotRun(m_operatorController.getLeftY()*-SpeedConstants.kPivotSpeed), pivotSubsystem));


    //-----------------------------------intake---------------------------
    m_operatorController.a().onTrue(new IntakeControl(IntakeSubsystem, TransitionSubsystem))
                .onFalse(Commands.runOnce(() -> IntakeSubsystem.intakeRun(0), IntakeSubsystem));

    //-------------------------------Reverse intake-------------------------
    m_operatorController.leftBumper().onTrue(new RunCommand(() -> IntakeSubsystem.intakeRun(-SpeedConstants.kIntakeSpeed), IntakeSubsystem))
                .onFalse(Commands.runOnce(() -> IntakeSubsystem.intakeRun(0), IntakeSubsystem));
            
  }

  public RobotContainer() {

    NamedCommands.registerCommand("Shoot", new RunCommand(() -> shooterSubsystem.shooterSub(SpeedConstants.kShootSubwoofer), shooterSubsystem));
    NamedCommands.registerCommand("Transition", new RunCommand(() -> TransitionSubsystem.transitionRun(SpeedConstants.kTransitionSpeed), TransitionSubsystem));
    NamedCommands.registerCommand("Intake", new IntakeControl(IntakeSubsystem, TransitionSubsystem));
    NamedCommands.registerCommand("Stop", Commands.runOnce(() -> shooterSubsystem.shooterSub(SpeedConstants.kNoShoot), shooterSubsystem));

    configureBindings();


    //Command runAuto = drivetrain.getAutoPath("testestAuto");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return runAuto;
    //return Commands.print("No autonomous command oooooooo configured");//amanda ong was here...the code works ...
  }
}
