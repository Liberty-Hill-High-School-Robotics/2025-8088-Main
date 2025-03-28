package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;



public class Climber extends SubsystemBase {

    private SparkMax climberSparkMax;

    public Climber(){
        //config motor settings here
        climberSparkMax = new SparkMax(CanIDs.climberMotorID, MotorType.kBrushless);
    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        //Mostly used for debug and such
    }

    public void climberUp() {
        climberSparkMax.set(MotorSpeeds.climberUpSpeed);
    }
    public void climberDown() {
        climberSparkMax.set(MotorSpeeds.climberDownSpeed);
    }
    public void climberStop() {
        climberSparkMax.set(0);
    }

}
