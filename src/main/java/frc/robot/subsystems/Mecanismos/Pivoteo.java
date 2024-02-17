package frc.robot.subsystems.Mecanismos;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Functions;

public class Pivoteo extends SubsystemBase {

    private final CANSparkMax motorIzq; 
    private final CANSparkMax motorDer; 
    
    public static Pivoteo instance;

    // Encoders:

    //private final AbsoluteEncoder encoderMotorIzq = motorIzq.getAbsoluteEncoder(Type.kDutyCycle);
    private final AbsoluteEncoder encoderAbs;

    public double output;

    // PID:
    ProfiledPIDController PID;


  /** Creates a new ExampleSubsystem. */
  public Pivoteo() {

    motorIzq= new CANSparkMax(10, MotorType.kBrushless);
    motorDer = new CANSparkMax(9, MotorType.kBrushless);

    motorDer.restoreFactoryDefaults();
    motorIzq.restoreFactoryDefaults();

    motorDer.setSmartCurrentLimit(35);
    motorIzq.setSmartCurrentLimit(35);


    encoderAbs = motorDer.getAbsoluteEncoder(Type.kDutyCycle);

    motorIzq.setInverted(true);


    // PID:
    PID = new ProfiledPIDController(1.35, 0, 0, new TrapezoidProfile.Constraints(22,22));
    PID.enableContinuousInput(0, 0.995);
    
    
  }


  public static Pivoteo getInstance(){
    if (instance == null){
        instance = new Pivoteo();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("AT GOAL", atGoal());
    SmartDashboard.putNumber("PID GOAL", PID.getGoal().position);
    SmartDashboard.putNumber("TOLERANCE", PID.getPositionTolerance());
    SmartDashboard.putNumber("TOLERANCE", PID.getPositionError());

    // K OFFSET = 0.0448

    SmartDashboard.putNumber("PID OUTPUT", output);
    SmartDashboard.putNumber("ENCODER VALUE", getPosition());

  }

  public void stopMotors(){
    motorDer.set(0);
    motorIzq.set(0);
  }

  public void setGoal(double goal){
    PID.setGoal(goal + 0.065);

    output = Functions.clamp(PID.calculate(encoderAbs.getPosition()), -0.5, 0.5);
    motorDer.set(output);
    motorIzq.set(output);

  }

  public double getAppliedOutput(){
    return motorDer.getAppliedOutput();

  }

  public double getPosition(){
    return encoderAbs.getPosition();
  }

  public void setVelocity(double speed){
    motorDer.set(speed);
    motorIzq.set(speed);

  }

  public boolean atGoal(){
    return PID.atGoal();
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

