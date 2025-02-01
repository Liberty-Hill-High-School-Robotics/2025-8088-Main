package frc.robot.commands.Algae;
import frc.robot.subsystems.Algae;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class IntakeIn extends Command {
  // The subsystem the command runs on
  private final Algae m_algae;

  public IntakeIn(Algae subsystem) {
    m_algae = subsystem;
    addRequirements(m_algae);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){

  }

  @Override 
  public void end(boolean interrupted){

   }

  @Override
  public boolean isFinished() {
    return false;
  }
}