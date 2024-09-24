// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class driveTrain extends SubsystemBase {
  private CANSparkMax frontR, backR, frontL, backL;
  /** Creates a new driveTrain. */
  public driveTrain() {
    frontR = new CANSparkMax(0, MotorType.kBrushless);
    frontL = new CANSparkMax(1, MotorType.kBrushless);
    backR = new CANSparkMax(2, MotorType.kBrushless);
    backL = new CANSparkMax(3, MotorType.kBrushless);
    backL.follow(frontL);
    backR.follow(frontR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double forward ,double turn){
    double leftSpd = forward + turn;
    double rightSpd = forward - turn;

    frontL.set(leftSpd);
    frontR.set(rightSpd);
  }
}
