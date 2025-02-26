package frc.robot.commands.Coral;
import frc.robot.subsystems.Coral;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class IntakeOut extends Command {
  // The subsystem the command runs on
  private final Coral m_coral;

  public IntakeOut(Coral subsystem) {
    m_coral = subsystem;
    addRequirements(m_coral);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    m_coral.coralOut();

  }

  @Override 
  public void end(boolean interrupted){
    m_coral.coralStop();
   }

  @Override
  public boolean isFinished() {
    return !m_coral.coralGetBeam();
  }
}