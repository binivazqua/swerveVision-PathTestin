// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mecanismos;

import frc.robot.subsystems.Mecanismos.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeButtonCmd extends Command {
  private final IntakeSubsystem intake_subsystem;

  private final double speed;

  private boolean detectarNote = false; 
  
  public IntakeButtonCmd(double speed) {
    intake_subsystem = IntakeSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake_subsystem);
    this.speed = speed;
  }

  public IntakeButtonCmd(double speed, boolean detectarNote) {
    intake_subsystem = IntakeSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake_subsystem);
    this.speed = speed;
    this.detectarNote = detectarNote;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake Button Command Started!");
    intake_subsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake_subsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake_subsystem.setSpeed(0);
    System.out.println("Intake Button Command Ended Succesfully!");    
    intake_subsystem.encenderLeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake_subsystem.detectedNote() == true && detectarNote == true)
      return true;
    else     
      return false;
    //return intake_subsystem.detectedNote();
  }
}