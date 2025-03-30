package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;



public class Climber extends SubsystemBase {

    private SparkFlex climberSparkFlex;
    private SparkClosedLoopController climberPID;
    

    public Climber(){
        //config motor settings here
        climberSparkFlex = new SparkFlex(CanIDs.climberMotorID, MotorType.kBrushless);
        climberPID = climberSparkFlex.getClosedLoopController();
        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(MotorSpeeds.cP, MotorSpeeds.cI, MotorSpeeds.cD, 1/565); //reciprocal of the motor's velocity constant
        climberSparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
        SmartDashboard.putNumber("Climber Velocity", climberSparkFlex.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        //Mostly used for debug and such
    }

    public void climberUp() {
        climberPID.setReference(MotorSpeeds.climberUpSpeed, ControlType.kVelocity);
    }
    public void climberDown() {
        climberPID.setReference(-   MotorSpeeds.climberDownSpeed, ControlType.kVelocity);
    }
    public void climberStop() {
        climberSparkFlex.set(0);
    }

}
