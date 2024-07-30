// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.MusicCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // ******************** AUTO-GENERATED SWERVE CODE FROM PHOENIX TUNER ********************
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
  // ******************** AUTO-GENERATED SWERVE CODE FROM PHOENIX TUNER ********************

  // Define Robot Subsystems
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Orchestra for playing music through CTRE motor controllers
  private final Orchestra orchestra = new Orchestra();

  // Define Robot Commands
  private final MusicCommand music = new MusicCommand(orchestra);


  private void configureBindings() {
    // ******************** AUTO-GENERATED SWERVE CODE FROM PHOENIX TUNER ********************
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    // ******************** AUTO-GENERATED SWERVE CODE FROM PHOENIX TUNER ********************
 
    // Schedule setTrapezoidalGoalState command when Xbox controller's X button is pressed
    // Cancel setTrapezoidalGoalState command when Xbox controller's X buttonn is released
    // TODO: update how many rotations on button press
    joystick.x().onTrue(armSubsystem.setTrapezoidGoalState(0.5));
    // Same command with Y button, moves the arm to 0 position
    joystick.y().onTrue(armSubsystem.setTrapezoidGoalState(0));

    // Press left trigger to play a song
    joystick.leftTrigger().onTrue(music);

  }

  private void initializeOrchestra(){
    // Add instruments to the orchestra (each motor controlled by a CTRE controller can be an instrument)
    // TODO: Add more instruments :)
    orchestra.addInstrument(armSubsystem.getTalonFX_L());
    orchestra.addInstrument(armSubsystem.getTalonFX_R());
    // Attempt to load the chrp file and log an error if it doesn't work
    var chrp_status = orchestra.loadMusic("music/UnderTheSea.chrp");
    if (!chrp_status.isOK()) {
      // TODO: log error here, idk how to do that yet
    }
  }

  // RobotContainer Constructor
  public RobotContainer() {
    configureBindings();
    initializeOrchestra();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

