package frc.robot.commands.Elevator;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ElevatorStop extends Command {
  // The subsystem the command runs on
  private final Elevator m_elevator;

  public ElevatorStop(Elevator subsystem) {
    m_elevator = subsystem;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    m_elevator.elevatorStop();
  }

  @Override 
  public void end(boolean interrupted){

   }

  @Override
  public boolean isFinished() {
    return false;
  }
}