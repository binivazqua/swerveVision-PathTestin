// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mecanismos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  PowerDistribution pdh = new PowerDistribution();

  private final CANSparkMax motorIntake;
  private final RelativeEncoder encoderIntake;

  private final I2C.Port i2cPort;

  private final ColorSensorV3 m_colorSensor;

  private boolean detectedred;
  private boolean detectedgreen;
  private boolean detectedblue;

  private Color detectedColor;

  public IntakeSubsystem() {
    /** Initialization for the motor */
    motorIntake = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
    motorIntake.restoreFactoryDefaults();
    motorIntake.setInverted(IntakeConstants.kMotorInverted);
    motorIntake.setIdleMode(IntakeConstants.kMotorIdleMode);

    i2cPort = I2C.Port.kOnboard;

    m_colorSensor = new ColorSensorV3(i2cPort);

    detectedColor = m_colorSensor.getColor();

    /** Initialization for the relative encoder */
    encoderIntake = motorIntake.getEncoder();
    resetEncoder();
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

  public void encenderLeds(){
    pdh.setSwitchableChannel(detectedNote());
  }

  public boolean detectedNote(){
    if (detectedColor.red > .30 && detectedColor.red < .55){
      detectedred = true;
    } else{
      detectedred= false;
    }

    if (detectedColor.green > .35 && detectedColor.green < .48){
      detectedgreen =true;
      
    } else {
      detectedgreen= false;
  
    }

   if (detectedColor.blue > .10 && detectedColor.blue < .23){
      detectedblue =true;
      
    } else {
      detectedblue= false;
  
    }

    return detectedblue && detectedgreen && detectedred;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Intake", getEncoder());
    SmartDashboard.putBoolean("Note loaded?", detectedNote());
    SmartDashboard.putBoolean("LEDS ON?", pdh.getSwitchableChannel());


    detectedColor = m_colorSensor.getColor();
  }

  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance(){
    if(instance == null){
      instance = new IntakeSubsystem();
    }
    return instance;
  }


}