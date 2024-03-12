package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonLL extends SubsystemBase {
  //PhotonCamera camera = new PhotonCamera("camara1");
  //PhotonCamera camera = new PhotonCamera("camara1");
  PhotonCamera camera = new PhotonCamera("camara2");


  private double yaw;

  private double pitch;

  private double area;


  private double Id;

  private double xMeters;

  private double yMeters;

  private boolean hasTargets;
  


      
    
  /** Creates a new ExampleSubsystem. */
  public PhotonLL() {
   

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets){
      var target = result.getBestTarget();
  
      // GET DATA:
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      Id = target.getFiducialId();
      Transform3d camToTarget = target.getBestCameraToTarget();
      
      xMeters = camToTarget.getX();
      yMeters = camToTarget.getY();

    /* 
      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch", pitch);
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putNumber("Apriltag Id", Id);
*/
      SmartDashboard.putNumber("Y-METERS", yMeters);
            SmartDashboard.putNumber("X-METERS", xMeters);

      SmartDashboard.putNumber("X ANGLE", camToTarget.getRotation().getX());
      SmartDashboard.putNumber("Y ANGLE", -camToTarget.getRotation().getY());
      SmartDashboard.putNumber("Z ANGLE", camToTarget.getRotation().getZ());

    } 

  }

    private static PhotonLL instance;
    public static  PhotonLL getInstance(){
      if (instance == null){
        instance = new PhotonLL();
      }
      
      return instance;
    }
    
    public boolean hasValueTargets(){
      return hasTargets;
    }

    public double getYaw(){
      return yaw;
    }

    public double getYDistance(){
      return yMeters;
    }

    public double getXDistance(){
      return xMeters;
    }

    public double getArea(){
      return area;
    }




  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

