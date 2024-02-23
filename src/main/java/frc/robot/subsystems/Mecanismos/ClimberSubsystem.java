package frc.robot.subsystems.Mecanismos;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    public static Solenoid solenoide;


    
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {

    solenoide = new Solenoid(PneumaticsModuleType.REVPH, 8);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Escalando?", solenoide.get());
  }

  
  public void climb (Boolean escala){
    solenoide.set(escala);  
  }

  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance(){
    if (instance == null){
        instance = new ClimberSubsystem();
    }
    return instance;
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
