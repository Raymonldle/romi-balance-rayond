// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.RomiGyro;

/** An example command that uses an example subsystem. */
public class straight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_drivebase;
  private final RomiGyro m_RomiGyro;
  PIDController m_PIDController = new PIDController(0.05, 0, 0);
  private boolean isDetected = false;
  private final int m_distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public straight(RomiDrivetrain m_drivebase, RomiGyro m_RomiGyro, int m_distance) {
    this.m_drivebase = m_drivebase;
    this.m_RomiGyro = m_RomiGyro;
    this.m_distance = m_distance;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_drivebase.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.arcadeDrive(m_PIDController.calculate(m_RomiGyro.getAngleX(),0),m_PIDController.calculate(m_RomiGyro.getAngleZ(),0));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drivebase.arcadeDrive(0 , 0);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivebase.getAvgDistanceInch() >= m_distance;
  }
}
