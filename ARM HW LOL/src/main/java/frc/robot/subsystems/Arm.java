// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Arm() {}

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

 CANSparkMax m_motor1 = new CANSparkMax(1, MotorType.kBrushless);
 RelativeEncoder m_encoder1 = m_motor1.getEncoder(Type.kQuadrature, 8192);

 CANSparkMax m_motor2 = new CANSparkMax(2, MotorType.kBrushless);
 RelativeEncoder m_encoder2 = m_motor1.getEncoder(Type.kQuadrature, 8192);

 PIDController m_PIDController = new PIDController(0.1, 0.1, 0.1);
 PIDController m_2PIDController = new PIDController(0.1, 0.1, 0.1);

double SPX;
double SPY;

double SPY1;
double SPX1;

 public void changeAngle(double SPy, double SPx) {
  SPX = SPx;
  SPY = SPy;
  double theta1 = 360 * m_encoder1.getPosition();
  double theta2 = Math.atan((SPY / SPX));
  double output = m_PIDController.calculate(theta1, theta2);
  SmartDashboard.putNumber("Angle", theta1);
  m_motor1.set(output);
 }

 public void extendLength(double SPy, double SPx) {
  SPX1 = SPx;
  SPY1 = SPy;
  double setPoint = Math.sqrt(Math.pow((SPX - 0), 2) + Math.pow((SPY - 3), 2));
  double r2Length = 3 * (m_encoder2.getPosition()) + 2;
  double r3Length = r2Length + Constants.ElevatorConstants.r1;
  double howMuchToMove = m_2PIDController.calculate(r3Length, setPoint);
  m_motor2.set(howMuchToMove);
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
