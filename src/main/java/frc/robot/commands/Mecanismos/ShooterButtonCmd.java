// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mecanismos;

import frc.robot.subsystems.Mecanismos.IntakeSubsystem;
import frc.robot.subsystems.Mecanismos.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterButtonCmd extends Command {
  private final ShooterSubsystem shooter_subsystem;

  private final double speed;

  public ShooterButtonCmd(double speed) {
    shooter_subsystem = ShooterSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter_subsystem);

    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter Button Command Started!");
    shooter_subsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter_subsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_subsystem.setSpeed(0);
    System.out.println("Shooter Button Command Ended Succesfully!");    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    return false;
  }
}