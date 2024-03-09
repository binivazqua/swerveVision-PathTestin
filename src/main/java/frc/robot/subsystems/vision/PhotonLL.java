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

    
    public void filterAprilTags(){
      var result = camera.getLatestResult();
    
      
    
      if (result.hasTargets()) {
      //LIST of targets photon vision has
      var targets = result.getTargets();
  
      //checks to see if there is a list of apriltags to check. if no targets are visable, end command
        if (targets.isEmpty()) {
          SmartDashboard.putBoolean("done", true);
        }
  
       var foundTargets = targets.stream().filter(t -> t.getFiducialId() == 4 || t.getFiducialId() == 8)
                        .filter(t -> !t.equals(4) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                        .findFirst();
  
        if (foundTargets.isPresent()) {
       var cameraToTarget = foundTargets.get().getBestCameraToTarget();
  }}
    }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

