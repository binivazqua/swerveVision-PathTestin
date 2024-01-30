package frc.robot.commands;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class neitoCommand extends CommandBase {
  
  public static CANSparkMax neito = new CANSparkMax(9, MotorType.kBrushless);


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public neitoCommand() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Intake Started", isFinished());

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    neito.set(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Intake Ended", isFinished());
    neito.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
