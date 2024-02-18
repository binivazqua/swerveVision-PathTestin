// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mecanismos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax motorShooterDer;
  RelativeEncoder encoderShooterDer;

  CANSparkMax motorShooterIzq;
  RelativeEncoder encoderShooterIzq;

  public ShooterSubsystem() {
    /** Initialization for the motor */
    boolean invertingDer = true;
    IdleMode modoMotores = IdleMode.kBrake;

    motorShooterDer = new CANSparkMax(12, MotorType.kBrushless);
    motorShooterDer.restoreFactoryDefaults();
    motorShooterDer.setInverted(invertingDer);
    motorShooterDer.setIdleMode(modoMotores);
    
    motorShooterIzq = new CANSparkMax(13, MotorType.kBrushless);
    motorShooterIzq.restoreFactoryDefaults();
    motorShooterIzq.setInverted(invertingDer);
    motorShooterIzq.setIdleMode(modoMotores);

    /** Initialization for the relative encoder */
    encoderShooterDer = motorShooterDer.getEncoder();
    encoderShooterIzq = motorShooterIzq.getEncoder();

    resetEncoder();
  }

  /** Crea las funciones para las distintas cosas que puede hacer tu sistema **/
  public void setSpeed(double speed){
    motorShooterDer.set(speed);
    motorShooterIzq.set(speed);
    
  }

  public double getEncoderDer(){
    return encoderShooterDer.getPosition();
  }

  public double getEncoderIzq(){
    return encoderShooterIzq.getPosition();
  }

  public void resetEncoder(){
    encoderShooterDer.setPosition(0);
    encoderShooterIzq.setPosition(0);
  }

  public void stopMotor(){
    motorShooterDer.set(0);
    motorShooterIzq.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Shooter Derecho", getEncoderDer());
    SmartDashboard.putNumber("Encoder Shooter Izquierdo", getEncoderIzq());

  }

  private static ShooterSubsystem instance;

  public static ShooterSubsystem getInstance(){
    if(instance == null){
      instance = new ShooterSubsystem();
    }
    return instance;
  }


}
