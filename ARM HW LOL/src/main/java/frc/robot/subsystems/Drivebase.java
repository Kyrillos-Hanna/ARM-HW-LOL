// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.IllegalArgumentException;

public class Drivebase extends SubsystemBase {
    
  CANSparkMax m_leftMaster = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_leftSlave = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_rightMaster = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rightSlave  = new CANSparkMax(4, MotorType.kBrushless);

  MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightMaster, m_rightSlave);
  MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftMaster, m_leftSlave);
  DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  public final AHRS m_AHRS = new AHRS(Port.kMXP);
  PIDController m_PIDController = new PIDController(0.1, 0.1, 0.1);

  /** Creates a new ExampleSubsystem. */
  public Drivebase() {
    m_rightGroup.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void arcadeDrive(double speed, double rotation) {
    m_differentialDrive.arcadeDrive(speed, rotation);
  }

  public void turn(double angle) {
    m_differentialDrive.arcadeDrive(0, m_PIDController.calculate(m_AHRS.getAngle(), angle));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
