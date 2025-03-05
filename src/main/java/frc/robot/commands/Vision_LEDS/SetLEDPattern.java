package frc.robot.commands.Vision_LEDS;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class SetLEDPattern extends Command {
  // The subsystem the command runs on
  private final LED m_led;

  public SetLEDPattern(LED subsystem) {
    m_led = subsystem;
    addRequirements(m_led);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    m_led.DefaultSetPattern();
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}