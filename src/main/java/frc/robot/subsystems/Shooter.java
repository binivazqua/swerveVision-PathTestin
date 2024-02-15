package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static CANSparkMax motorIzq = new CANSparkMax(13, MotorType.kBrushless);
  public static CANSparkMax motorDer = new CANSparkMax(12, MotorType.kBrushless);


  public Shooter() {
    motorIzq.restoreFactoryDefaults();
    motorDer.restoreFactoryDefaults();

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public static void setMotors(double speed){
    motorIzq.set(speed);
    motorDer.set(speed);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

