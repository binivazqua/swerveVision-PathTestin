package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static CANSparkMax motor = new CANSparkMax(11, MotorType.kBrushless);

  public Intake() {
    motor.restoreFactoryDefaults();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public static void setMotors(double speed){
    motor.set(speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

