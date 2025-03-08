package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.commands.Coral.IntakeOut;
import frc.robot.commands.Elevator.ElevatorLevelEnd;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;


/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoL4 extends SequentialCommandGroup {


    public AutoL4(
        Coral m_coral,
        Elevator m_elevator
    ){
    
        addCommands(
            new ElevatorLevelEnd(m_elevator, MotorSpeeds.elevatorL2),
            new IntakeOut(m_coral)
            //new ElevatorDownDefault(m_elevator) //TODO: CHECK IF THIS GOES DOWN WHEN THE COMMAND IS NOT RUN!
            );
        }


    @Override
    public boolean runsWhenDisabled() {

        return false;
    }
}