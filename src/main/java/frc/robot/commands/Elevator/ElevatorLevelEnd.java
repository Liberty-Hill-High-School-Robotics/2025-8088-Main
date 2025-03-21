package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ElevatorLevelEnd extends Command {
  // The subsystem the command runs on
  private final Elevator m_elevator;
  double local;

  public ElevatorLevelEnd(Elevator subsystem, double setpoint) {
    m_elevator = subsystem;
    local = setpoint;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    m_elevator.elevatorL(local);
  }

  @Override 
  public void end(boolean interrupted){
    m_elevator.elevatorStop();
   }

  @Override
  public boolean isFinished() {
    if(m_elevator.elevatorAtPos(local)){
      return true;
    }
    else{
      return false;
    }
  }
}