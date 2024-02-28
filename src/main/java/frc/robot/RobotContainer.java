// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  private final TransitionSubsystem TransitionSubsystem = new TransitionSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  //private final ShooterSubsystem ShooterWoofSubsystem = new ShooterSubsystem();
  //private final ShooterSubsystem ShooterPodiumSubsystem = new ShooterSubsystem();
  //private final ShooterSubsystem ShooterAmpSubsystem = new ShooterSubsystem();

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

   // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
       // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
   

    //intake Buttons
    joystick.a().onTrue(new RunCommand(() -> IntakeSubsystem.intakeRun(Constants.SpeedConstants.kIntakeSpeed), IntakeSubsystem))
                .onFalse(Commands.runOnce(() -> IntakeSubsystem.intakeRun(0), IntakeSubsystem));
    
    //transition Buttons
    //joystick.b().onTrue(new RunCommand(() -> TransitionSubsystem.transitionRun(Constants.SpeedConstants.kTransitionSpeed)))
    //            .onFalse(Commands.runOnce(() -> TransitionSubsystem.transitionRun(0)));

    //Shooter Buttons-----------------------------------------------------------------------------------------------lots of them

    //shooter sub
    joystick.y().onTrue(new RunCommand(() -> shooterSubsystem.shooterSub(Constants.SpeedConstants.kShootSubwoofer), shooterSubsystem))
                .onFalse(Commands.runOnce(() -> shooterSubsystem.shooterSub(0), shooterSubsystem));

    //shooter podium
    joystick.x().onTrue(new RunCommand(() -> shooterSubsystem.shooterPodium(Constants.SpeedConstants.kShootPodium), shooterSubsystem))
                .onFalse(Commands.runOnce(() -> shooterSubsystem.shooterPodium(0), shooterSubsystem));
    //shooter amp
   /***  joystick.rightBumper().onTrue(new RunCommand(() -> ShooterAmpSubsystem.shooterAmp()))
                .onFalse(Commands.runOnce(() -> ShooterAmpSubsystem.shooterAmp(0)));
***/
            
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
