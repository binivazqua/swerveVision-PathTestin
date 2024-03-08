package frc.robot.commands.Mecanismos;

import frc.robot.subsystems.Mecanismos.Pivoteo;
import frc.robot.subsystems.vision.PhotonLL;
import edu.wpi.first.wpilibj2.command.Command;

public class PivoteoByInterpolation extends Command {
  private final Pivoteo arm;
    private final PhotonLL camera;
    private double key;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PivoteoByInterpolation() {
    arm = Pivoteo.getInstance();
    camera = PhotonLL.getInstance();
    
    
    addRequirements(arm);
    addRequirements(camera);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    key = camera.getArea();
    arm.setPositionByInterpolation(key);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("ENDED SUCCESSFULLY");
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


