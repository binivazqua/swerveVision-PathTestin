package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonLL extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("camera1");

  private double yaw;

  private double pitch;

  private double area;

  private double skew;

  


      
    
  /** Creates a new ExampleSubsystem. */
  public PhotonLL() {
   

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    if (result.hasTargets()){
      var target = result.getBestTarget();
  
      // GET DATA:
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();
      var camToTarget = target.getBestCameraToTarget();

      

      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch", pitch);
      SmartDashboard.putNumber("Area", area);


    
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

