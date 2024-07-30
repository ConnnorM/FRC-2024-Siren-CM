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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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
  // Follower - defines a master motor controller so that other controllers can be set to mimic
  // or reverse whatever the master controller does
  Follower follower;
  // Position Voltage - a control for the motor controllers that uses a setpoint to direct the motor
  // while also applying voltage as a feed forward value
  PositionVoltage positionVoltage;
  // Trapezoid Profile - a trapezoid motion profile that is intended to be combined with PID control
  TrapezoidProfile trapezoidProfile;
  // Trapezoid Profile State - the desired goal state of the arm
  TrapezoidProfile.State tp_goal_state;
  // Trapezoid Profile State - the current state of the arm
  TrapezoidProfile.State tp_current_state;


  /** Constructor: creates a new ArmSubsystem. Initialize and configure all of the objects
   * that you have previously defined for the ArmSubsystem
  */
  public ArmSubsystem() {
    // 1. Intialize the CANCoder and configuration object
    canCoder = new CANcoder(0);
    canCoder_config = new CANcoderConfiguration();

    // 2. Set CANCoder configuration variables
    // Set the range of values that the sensor outputs to be [-0.5, 0.5) rotations
    canCoder_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // Set which spin direction of the sensor is positive (when facing the LED side of the CANCoder)
    // to be counter clockwise
    canCoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // Add an offset to the reading from the sensor. Since the sensor is not mounted to the robot such that
    // we want a reading of 0 when the magnet aligns with the LED, we want to offset the reading so that
    // we get a reading of 0 when we are at an angle we define to be 0 (I think this is parallel with the ground?)
    // TODO: get this actual value using Phoenix Tuner while holding the arm at what we want 0 degrees to be
    canCoder_config.MagnetSensor.MagnetOffset = 0;
    // Apply the configuration to the CANCoder object
    canCoder.getConfigurator().apply(canCoder_config);

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

    // 5. Set our PID values for the TalonFX controllers using slotConfigs
    Slot0Configs slotConfigs = new Slot0Configs();
    // TODO: Set values for our PID constants
    slotConfigs.kP = 0;
    slotConfigs.kI = 0;
    slotConfigs.kD = 0;
    // Apply these PID settings (gains) to slot 0 on our TalonFX motor controllers
    talonFX_L.getConfigurator().apply(slotConfigs);

    // 6. Set the right TalonFX controller to follow the motor output of the left controller
    // Set the motor ID of the master motor. Since the motors both work to lift the arm and are
    // mounted on opposite sides of the robot, we need both motors to spin in opposite directions
    // in order to both lift/lower the arm together
    follower = new Follower(talonFX_L.getDeviceID(), true);
    // Tell the right TalonFX to follow the left TalonFX
    talonFX_R.setControl(follower);

    // 7. Initialize PositionVoltage control for TalonFX motor controller
    // PositionVoltage - set the position, in rotations, that we want to drive towards
    // when we intialize the armSubsystem (our initial/resting state, which is parallel to the ground)
    // withSlot - set which PID gains to apply to this control
    // withFeedForward - how much of a feed forward signal to apply in volts
    // TODO: why is feed forward 0? That means it does nothing right?
    // withEnableFOC - a Phoneix Pro Pay 2 Win feature: increases power by ~15% by leveraging
    // torque (current) control
    positionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0).withEnableFOC(true);

    // 8. Intialize our Trapezoidal Motion Profile
    // Create the trapezoidProfile and set max Velocity and Acceleration. Units are in rotations/second
    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(20, 40));
    // Create a goal state and a current state for the trapezoidProfile
    tp_goal_state = new TrapezoidProfile.State();
    tp_current_state = new TrapezoidProfile.State();
  }

  // Use positionVoltage control only to move the arm (no trapezoidal motion profile included)
  public Command setArmSetpoint(double setpoint) {
    // Pass in a lambda (unnamed, one-line) function into runOnce and return a Command
    // that that executes the lambda function
    return this.runOnce(
        () -> {
          // Request PID to move to target position
          // This overloaded setControl method takes in a positionVoltage object (which carries
          // a position and a feed forward voltage to apply)
          // Pass in the desired setpoint (position in rotations) to the positionVoltage object
          talonFX_L.setControl(positionVoltage.withPosition(setpoint));
          // Remember that talonFX_R used setControl(follower), so it will just mimic talonFX_L
        });
  }

  // Set the armSubsystem's trapezoid profile's goal state by feeding in a position (in rotations)
  // periodic will then check to see if we are at the goal, and if not, it will use 
  // positionVoltage to move the arm to that position according to the trapezoidal motion profile
  public Command setTrapezoidGoalState(double setpoint){
    return this.runOnce(
      () -> {
        // Upon reaching this state, the velocity should be 0
        tp_goal_state = new TrapezoidProfile.State(setpoint, 0);
      });
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Calculate the new current state (as time passes while the arm moves) using the trapezoidProfile
    // Returns the position and velocity of the current state
    tp_current_state = trapezoidProfile.calculate(0.02, tp_current_state, tp_goal_state);
    // Use the tp_current_state's position and velocity to update the positionVoltage control
    positionVoltage.Position = tp_current_state.position;
    positionVoltage.Velocity = tp_current_state.velocity;
    // Command the arm to move to the desired position using the values calculated by the trapezoid motion profile
    talonFX_L.setControl(positionVoltage);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Getters for motors for l'orquestre
  public TalonFX getTalonFX_L(){
    return this.talonFX_L;
  }
  public TalonFX getTalonFX_R(){
    return this.talonFX_R;
  }
}
