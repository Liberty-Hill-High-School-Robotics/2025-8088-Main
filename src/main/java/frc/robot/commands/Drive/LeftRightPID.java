package frc.robot.commands.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class LeftRightPID extends Command {
  // The subsystem the command runs on
  private final DriveSubsystem m_DriveSubsystem;
  double localvalue;

  public LeftRightPID(DriveSubsystem subsystem) {
    m_DriveSubsystem = subsystem;
    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    m_DriveSubsystem.FullPIDControl();
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}