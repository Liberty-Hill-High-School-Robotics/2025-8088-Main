package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.easyAlign;
import frc.robot.subsystems.DriveSubsystem;


/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class Align extends SequentialCommandGroup {


    public Align(
        DriveSubsystem m_drivesubsystem
    ){
    
        addCommands(
            new easyAlign(m_drivesubsystem, false).withTimeout(5)
            );
        }


    @Override
    public boolean runsWhenDisabled() {

        return false;
    }
}