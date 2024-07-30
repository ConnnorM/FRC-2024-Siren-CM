// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.minor_subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * MusicSubsystem
 * This system uses TalonFX motor controllers to play music. This class may seema bit
 * redundant, but the main idea is to turn our orchestra into a subsystem so that we
 * can have orchestra.play/pause/stop be commands that can be triggered by button inputs
 */
public class MusicSubsystem extends SubsystemBase {
  Orchestra m_orchestra;

  public MusicSubsystem() {
    m_orchestra = new Orchestra();
    // Load a chrp file into the orchestra
    var orchestra_status = m_orchestra.loadMusic("music/UnderTheSea.chrp");
    if (!orchestra_status.isOK()) {
    // TODO: log error here, idk how to make log messages yet
    }
  }

  // Add an instrument to the orchestra
  public void addInstrument(TalonFX talonFX) {
    m_orchestra.addInstrument(talonFX);
  }

  // A command that plays music
  public Command playMusic() {
    return runOnce(
        () -> {
          m_orchestra.play();
        });
  }

  // A command that pauses music
  public Command pauseMusic() {
    return runOnce(
        () -> {
          m_orchestra.pause();
        });
  }

  // A command that stops music
  public Command stopMusic() {
    return runOnce(
        () -> {
          m_orchestra.stop();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

