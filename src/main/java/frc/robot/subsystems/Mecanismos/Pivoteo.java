package frc.robot.subsystems.Mecanismos;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Functions;
import frc.robot.Constants;
import frc.robot.Constants.PivotingConstants;

public class Pivoteo extends SubsystemBase {

    private final CANSparkMax leftMotor; 
    private final CANSparkMax rightMotor; 
    
    public static Pivoteo instance;

    PowerDistribution pdh = new PowerDistribution();

    RelativeEncoder encoder;
    // Encoders:
    //private final AbsoluteEncoder encoderMotorIzq = motorIzq.getAbsoluteEncoder(Type.kDutyCycle);
    private final AbsoluteEncoder absoluteEncoder;

    public double output;
    

    // PID:
    ProfiledPIDController PID;
    InterpolatingDoubleTreeMap goalMap;

  /** Creates a new ExampleSubsystem. */
  public Pivoteo() {

    leftMotor= new CANSparkMax(PivotingConstants.kLeftMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(PivotingConstants.kRightMotorID, MotorType.kBrushless);

    rightMotor.restoreFactoryDefaults();
    rightMotor.setSmartCurrentLimit(PivotingConstants.kMotorsCurrentLimit);

    leftMotor.restoreFactoryDefaults();
    leftMotor.setSmartCurrentLimit(PivotingConstants.kMotorsCurrentLimit);

    absoluteEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

//ENCODER RELATIVO DEL MOTOR
 
    encoder = leftMotor.getEncoder();
    encoder.setPosition(0); //-0.468 * 20

//THROUGH BORE COMO RELATIVO
   /*  encoder = rightMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    encoder.setPosition(0.448);
*/
    
    leftMotor.setInverted(PivotingConstants.kLeftMotorInverted);

    // PID:
    PID = new ProfiledPIDController(
    PivotingConstants.kP, 
    PivotingConstants.kI, 
    PivotingConstants.kD, 
      new TrapezoidProfile.Constraints(
        PivotingConstants.kMaxVelocity,
        PivotingConstants.kMaxAcceleration));

    goalMap = new InterpolatingDoubleTreeMap();
    goalMap.put(1.0, 0.20);
    goalMap.put(1.5, 0.22);
    goalMap.put(2.0, 0.26);




      
    
    /*PID.enableContinuousInput(
      0, 
      PivotingConstants.kPIDMaxInput);
    */
    
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
    SmartDashboard.putBoolean("LEEEEEDS", pdh.getSwitchableChannel());
    atGoal();

    // K OFFSET = 0.0448

    SmartDashboard.putNumber("PIVOT VALUE", getPosition());
    SmartDashboard.putNumber("PIVOT OUTPUT", getAppliedOutput());

    SmartDashboard.putNumber("APPLIED CURRENT PIVOT", leftMotor.getOutputCurrent());

    SmartDashboard.putNumber("ENCODER RELATIVO PIVOTEO", encoder.getPosition()/20);
    //encenderLeds();
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }
  public void encenderLeds(){
    pdh.setSwitchableChannel(atGoal());
  }

  public void stopMotors(){
    rightMotor.set(0);
    leftMotor.set(0);
  }

  
  public void setGoal(double goal){
    PID.setGoal(goal + 0.018);

    output = Functions.clamp(PID.calculate(encoder.getPosition()/20), -0.2, 0.2);
    rightMotor.set(output);
    leftMotor.set(output);

  }
/* 
  public void setGoal(double goal, double key){
    PID.setGoal(goalMap.get(key));
    output = Functions.clamp(PID.calculate(encoder.getPosition()/20), -0.2, 0.2);
    rightMotor.set(output);
    leftMotor.set(output);
    
  }*/

  public void setPositionByInterpolation(double key){
    //setGoal(goalMap.get(key));
    SmartDashboard.putNumber("Camera distance: ", key);
    SmartDashboard.putNumber("Pivot By Interpolation Goal: ", goalMap.get(key));
   


  }
  
  public void setAbsoluteEncoderZero(double zero){
    absoluteEncoder.setZeroOffset(zero);
  }

  public double getAppliedOutput(){
    return rightMotor.getAppliedOutput();

  }

  public double getPosition(){
    return encoder.getPosition()/20;
  }

  public void setVelocity(double speed){
    rightMotor.set(speed);
    leftMotor.set(speed);

  }


  public boolean atGoal(){
    boolean atGoal;

    //double position = PID.getGoal().position - 0.015;
    //0.12 a 55
    //0.1 a 60
    if(getPosition() <= (0.04) && getPosition() >=  0.01 ){
      atGoal = true;
    } else {
      atGoal = false;
    }

    return atGoal;
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

