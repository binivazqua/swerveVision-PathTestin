package frc.robot.subsystems.Mecanismos;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Functions;
import frc.robot.Constants;
import frc.robot.Constants.PivotingConstants;

public class Pivoteo extends SubsystemBase {

    private final CANSparkMax leftMotor; 
    private final CANSparkMax rightMotor; 
    
    public static Pivoteo instance;

    // Encoders:
    //private final AbsoluteEncoder encoderMotorIzq = motorIzq.getAbsoluteEncoder(Type.kDutyCycle);
    private final AbsoluteEncoder absoluteEncoder;

    public double output;

    // PID:
    ProfiledPIDController PID;

  /** Creates a new ExampleSubsystem. */
  public Pivoteo() {

    leftMotor= new CANSparkMax(PivotingConstants.kLeftMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(PivotingConstants.kRightMotorID, MotorType.kBrushless);

    rightMotor.restoreFactoryDefaults();
    leftMotor.restoreFactoryDefaults();

    rightMotor.setSmartCurrentLimit(PivotingConstants.kMotorsCurrentLimit);
    leftMotor.setSmartCurrentLimit(PivotingConstants.kMotorsCurrentLimit);


    absoluteEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftMotor.setInverted(PivotingConstants.kLeftMotorInverted);


    // PID:
    PID = new ProfiledPIDController(
    PivotingConstants.kP, 
    PivotingConstants.kI, 
    PivotingConstants.kD, 
      new TrapezoidProfile.Constraints(
        PivotingConstants.kMaxVelocity,
        PivotingConstants.kMaxAcceleration));

    PID.enableContinuousInput(
      PivotingConstants.kPIDminInput, 
      PivotingConstants.kPIDMaxInput);
    
    
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
    rightMotor.set(0);
    leftMotor.set(0);
  }

  public void setGoal(double goal){
    PID.setGoal(goal + 0.065);

    output = Functions.clamp(PID.calculate(absoluteEncoder.getPosition()), -0.2, 0.2);
    rightMotor.set(output);
    leftMotor.set(output);

  }

  public double getAppliedOutput(){
    return rightMotor.getAppliedOutput();

  }

  public double getPosition(){
    return absoluteEncoder.getPosition();
  }

  public void setVelocity(double speed){
    rightMotor.set(speed);
    leftMotor.set(speed);

  }

  public boolean atGoal(){
    return PID.atGoal();
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

