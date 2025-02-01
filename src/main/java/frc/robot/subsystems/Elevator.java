package frc.robot.subsystems;

//all imports here
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;



public class Elevator extends SubsystemBase {

    //motors & variables here, define them and create any PIDs needed
    /* ex:
    private CANSparkMax barRotatorSparkMax;
    private SparkLimitSwitch barReverseLimitSwitch;
    public static RelativeEncoder barRotatorRelativeEncoder;
    PIDController barPID = new PIDController(BarConstants.bP, BarConstants.bI, BarConstants.bD);
    */

    public Elevator(){
        //config motor settings here

        /*
        ex:
        barRotatorSparkMax = new CANSparkMax(CanIDs.barRotatorID, MotorType.kBrushless);
        //barRotatorSparkMax.restoreFactoryDefaults();
        barRotatorSparkMax.setInverted(true);
        barRotatorSparkMax.setIdleMode(IdleMode.kBrake);
        barRotatorSparkMax.setSmartCurrentLimit(40);

        barReverseLimitSwitch = barRotatorSparkMax.getReverseLimitSwitch(Type.kNormallyOpen);

        barRotatorSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
        barRotatorSparkMax.setSoftLimit(SoftLimitDirection.kForward, BarConstants.fLimit);

        barRotatorRelativeEncoder = barRotatorSparkMax.getEncoder();
        */
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

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    // Should include run/stop/run back, etc.

    //as well as check for limits and reset encoders,
    //return true/false if limit is true, or encoder >= x value

    public void ___(){
        //motor.set(PID.calculate(position, setpoint));
        //motor.set(number);
        
    }

}
