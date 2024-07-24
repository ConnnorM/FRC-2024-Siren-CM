// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.abstractMotorInterfaces.VortexMotorController;

/**
 * ArmSubsystem
 * This system controls the rotating shooter arm on the robot. It uses two
 * Kraken X60 motors working in tandem, backed by the TalonFX motor controller, to power 
 * its rotation. An absolute encoder (CANCoder) is mounted on the axle of the arm and 
 * measures the exact position of the arm.
 */

public class ArmSubsystem extends SubsystemBase {
  // First, define all of the objects that you will want as part of this subsystem
  // TalonFX Motor Controller - Left
  TalonFX talonFX_L;
  // TalonFX Motor Controller - Right
  TalonFX talonFX_R;
  // TalonFX Motor Controller - Configuration
  TalonFXConfiguration talonFX_config;
  // CANCoder - Absolute encoder to measure arm position
  CANcoder canCoder;
  // CANCoder - Configuration
  CANcoderConfiguration canCoder_config;
  // Follower - 
  Follower follower;
  // Position Voltage - 
  PositionVoltage positionVoltage;
  // Trapezoid Profile - 
  TrapezoidProfile trapezoidProfile;
  // Trapezoid Profile State - 
  TrapezoidProfile.State tp_goal;
  // Trapezoid Profile State - 
  TrapezoidProfile.State tp_setpoint;


  /** Constructor: creates a new ArmSubsystem. Initialize all of the objects
   * that you have previously defined for the ArmSubsystem
  */
  public ArmSubsystem() {
    // 1. Intialize the CANCoder and configuration object

    // 2. Set CANCoder configuration variables
    
    // 3. Initialize the motors and configuration object
    talonFX_L = new TalonFX(0);
    talonFX_R = new TalonFX(1);
    talonFX_config = new TalonFXConfiguration();
    // 4. Set TalonFX configuration variables
    // Set the motor controller state when there is no output (no commands being sent)
    // or when the robot is disabled to be Brake mode (holds current position)
    talonFX_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Set the maximum current that can be supplied to the motor to be 40 amps
    talonFX_config.CurrentLimits.SupplyCurrentLimit = 40;
    // Tell the controller what type of sensor to use for closed loop feedback
      // Remember that we use the CANCoder to measure the exact rotational position
      // of the arm, and we use the CANCoder's reading to inform the motor controllers of where the arm is
      // in real life so that the controllers know how close we are to the goal state
      // This is a PhoneixPro function that combines the TalonFX's internal rotor sensor with data from
      // a CANCoder
    talonFX_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // Set the device ID of the feedback sensor (CANCoder) that creates the closed loop feedback system
    talonFX_config.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    // Set the gear ratio of the sensor the actual mechanism that is moving
      // Since our sensor (CANCoder) is mounted to the axle of the arm, we set this ratio to 1.0
      // as 1 full rotation read by the sensor is equivalent to 1 full rotation of the mechanism we are moving
    talonFX_config.Feedback.SensorToMechanismRatio = 1.0;
    // Set the gear ratio from the motor to the actual sensor
      // Since there is gearing between the motor shaft and the location where the sensor is mounted,
      // we set this value to be that gear ratio
      // TODO: get actual gear ratio
    talonFX_config.Feedback.RotorToSensorRatio = 42.4286;
    // Apply the configuration values that we just set to our actual motor controller object
    talonFX_L.getConfigurator().apply(talonFX_config);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
