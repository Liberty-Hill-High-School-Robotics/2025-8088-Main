package frc.robot.commands.Vision;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class GetTargetYaw extends Command {
  // The subsystem the command runs on
  private final Vision m_Vision;

  public GetTargetYaw(Vision subsystem) {
    m_Vision = subsystem;
    addRequirements(m_Vision);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    m_Vision.getYaw();
  }

  @Override 
  public void end(boolean interrupted){

   }

  @Override
  public boolean isFinished() {
    return false;
  }
}