// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


public class Auto_Complex extends  SequentialCommandGroup {
  /** Creates a new Auto_Complex. */
  public Auto_Complex(Drivetrain dt) {
    addCommands(
      //drive forward 3 meters at 25% speed
      new Auto_DriveForward(3,0.25, dt),
      //drive back 2 meters at 50% speed
      new Auto_DriveForward(-2, 0.50, dt),
      //drive forward 4 meters 75% speed
      new Auto_DriveForward(4, 0.75, dt)
    );
  }
}