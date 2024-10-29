// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkRelativeEncoder;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intake;
  public Intake() {
    intake = new CANSparkMax(6, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // commands
  public void intake(){
    intake.set(.3);
  }
  public void outtake(){
    intake.set(-0.3);
  }
  public void hold(){
    intake.set(0);
  }
}