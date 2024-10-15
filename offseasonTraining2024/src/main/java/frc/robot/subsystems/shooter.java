// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class shooter extends SubsystemBase {
  private CANSparkMax leftM, rightM;
  /** Creates a new shooter. */

  public shooter() {
    leftM = new CANSparkMax(4, MotorType.kBrushless);
    rightM = new CANSparkMax(5, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void move(double speed) {

    leftM.set(speed);
    rightM.set(speed);

  }
  
}
