package frc.robot.commands.Mecanismos;

import frc.robot.subsystems.Mecanismos.ClimberSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
  private final ClimberSubsystem climber;
  private Boolean escala;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCommand(Boolean escala) {
    climber = ClimberSubsystem.getInstance();
    this.escala = escala;

    
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climb(escala);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
