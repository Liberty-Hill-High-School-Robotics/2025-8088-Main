package frc.robot.commands.Climber;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ClimbDown extends Command {
  // The subsystem the command runs on
  private final Climber m_climber;

  public ClimbDown(Climber subsystem) {
    m_climber = subsystem;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    m_climber.climberDown();
  }

  @Override 
  public void end(boolean interrupted){
    m_climber.climberStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}