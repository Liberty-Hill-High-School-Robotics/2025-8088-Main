package frc.robot.commands.Drive;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
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
  List<Trajectory.State> localvalue;

  public LeftRightPID(DriveSubsystem subsystem, List<Trajectory.State> splines) {
    m_DriveSubsystem = subsystem;
    localvalue = splines;
    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    m_DriveSubsystem.FullPIDControl(localvalue);
    System.out.println("running");
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}