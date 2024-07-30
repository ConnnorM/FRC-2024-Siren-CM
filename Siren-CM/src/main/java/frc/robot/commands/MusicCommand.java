/* -------------------- WARNING --------------------
 * This command has been deprecated. I don't know if this works at all since I scrapped
 * this command in favor of creating the more versatile MusicSubsystem.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.Orchestra;

public class MusicCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  Orchestra m_orchestra;

  public MusicCommand(Orchestra orchestra) {
    this.m_orchestra = orchestra;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_orchestra.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_orchestra.pause();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
