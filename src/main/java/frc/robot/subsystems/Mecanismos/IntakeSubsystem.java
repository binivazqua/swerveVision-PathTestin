// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mecanismos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax motorIntake;
  private final RelativeEncoder encoderIntake;

  private boolean inRange;

  private int proximity;

  DigitalInput infrared;



  public IntakeSubsystem() {
    /** Initialization for the motor */
    motorIntake = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
    motorIntake.restoreFactoryDefaults();
    motorIntake.setInverted(IntakeConstants.kMotorInverted);
    motorIntake.setIdleMode(IntakeConstants.kMotorIdleMode);

    
    /** Initialization for the relative encoder */
    encoderIntake = motorIntake.getEncoder();
    resetEncoder();
    
    /** Infrared sensor initialization **/
    infrared = new DigitalInput(1);

  }

  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance(){
    if(instance == null){
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  /** Crea las funciones para las distintas cosas que puede hacer tu sistema **/
  public void setSpeed(double speed){
    motorIntake.set(speed);
  }

  public double getEncoder(){
    return encoderIntake.getPosition();
  }

  public void resetEncoder(){
    encoderIntake.setPosition(0);
  }

  public void stopMotor(){
    motorIntake.set(0);
  }

  public boolean getInfrared(){
    if(!infrared.get() == true)
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //detectedColor = m_colorSensor.getColor();

   
    // INTAKE INFO

    SmartDashboard.putNumber("NOTE INTAKE: ", getEncoder());
    //SmartDashboard.putBoolean("LEDS ON?", pdh.getSwitchableChannel());
    SmartDashboard.putBoolean("NOTE LOADED: ", !getInfrared());

  }
/* 
  public int getProximity() {
    return m_colorSensor.getProximity();
  }

  public boolean noteColorDetected(){

    if (detectedColor.red > 0.30 && detectedColor.red < 0.55){
      detectedred = true;
    } else{
      detectedred = false;
    }

    if (detectedColor.green > 0.39 && detectedColor.green < 0.45){
      detectedgreen = true;
      
    } else {
      detectedgreen = false;
  
    }

   if (detectedColor.blue > 0.10 && detectedColor.blue < 0.23){
      detectedblue = true;
      
    } else {
      detectedblue= false;
  
    }
    

    return detectedblue && detectedgreen && detectedred;

  }

  public boolean detectedNote(){
    
    return noteInRange();
  }

  public boolean noteInRange(){
    if (getProximity() > 100 && getProximity() < 250){
      inRange = true;
    } else {
      inRange = false;
    }

    return inRange;

  }*/

 
  


}