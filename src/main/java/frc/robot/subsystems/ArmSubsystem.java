// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem implements AutoCloseable {
  private final CANSparkMax motor =
  new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
 
  private double voltageCommand = 0.0;
  private double goalPosition;

  private double encoderSimDistance;
  private double encoderSimRate;
  private double simCurrent;

 /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(new ProfiledPIDController(
      ArmConstants.kP,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ArmConstants.kMaxVelocityRadPerSecond,
          ArmConstants.kMaxAccelerationRadPerSecSquared)),
      0);
        
    encoder.setPositionConversionFactor(ArmConstants.kArmRadiansPerEncoderRotation);
    encoder.setVelocityConversionFactor(ArmConstants.kRPMtoRadPerSec);

    resetPosition();

    motor.setIdleMode(IdleMode.kBrake);
    motor.setVoltage(0.0);

    // Assume the arm is starting in the back rest position
    setGoal(ArmConstants.kArmOffsetRads);

    setupShuffleboard();

  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
    }
    updateShuffleboard();

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double newFeedforward = 0;
    if (m_enabled) {
      // Calculate the feedforward from the setpoint
      newFeedforward = feedforward.calculate(setpoint.position, setpoint.velocity);
      // Add the feedforward to the PID output to get the motor output
      voltageCommand = output + newFeedforward;
    }
    else {
      voltageCommand = 0;
    } 
    motor.setVoltage(voltageCommand);

    SmartDashboard.putNumber("feedforward", newFeedforward);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("SetPt Pos", Units.radiansToDegrees(setpoint.position));
    SmartDashboard.putNumber("SetPt Vel", Units.radiansToDegrees(setpoint.velocity));

  }

  @Override
  // Arm position for PID measurement (Radians relative to horizontal)
  public double getMeasurement() {
    if (RobotBase.isReal()) {
      return encoder.getPosition() + ArmConstants.kArmOffsetRads; // Add offset for starting zero point
    }
    else {
      return encoderSimDistance + ArmConstants.kArmOffsetRads; // Add offset for starting zero point
    }
  }

  // Motor speed (Rad/sec)
  public double getVelocity() {
    if (RobotBase.isReal()) {
      return encoder.getVelocity();
    }
    else {
      return encoderSimRate;
    }
  }

  // Motor current (Amps)
  public double getCurrent() {
    if (RobotBase.isReal()) {
      return motor.getOutputCurrent();
    }
    else {
      return simCurrent;
    }
  }

  // Motor Commanded Voltage
  public double getVoltageCommand() {
    return voltageCommand;
  }

  // Set encoder distance for use in simulation
  public void setSimDistance(double distance) {
    encoderSimDistance = distance;
  }

  // Set encoder rate for use in simulation
  public void setSimRate(double rate) {
    encoderSimRate = rate;
  }

  // Set motor current for use in simulation
  public void setSimCurrent(double current) {
    simCurrent = current;
  }

  // Reset the encoder to zero. Should only be used when arm is in neutral offset position.
  public void resetPosition() {
    // Arm position for PID measurement
      encoder.setPosition(0) ;
  }

   // Calculate increased  goal limited to allowed range
   public double increasedGoal() {
    double newGoal = m_controller.getGoal().position + Constants.ArmConstants.kPosIncrement;
    return MathUtil.clamp(newGoal, Constants.ArmConstants.kMinAngleRads, Constants.ArmConstants.kMaxAngleRads);
  }

  // Calculate decreased  goal limited to allowed range
  public double decreasedGoal() {
    double newGoal =  m_controller.getGoal().position - Constants.ArmConstants.kPosIncrement;
    return MathUtil.clamp(newGoal, Constants.ArmConstants.kMinAngleRads, Constants.ArmConstants.kMaxAngleRads);

  } 
  @Override
  /** Enables the PID control. Resets the controller. */
  public void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!m_enabled || Constants.allowReenable) {
      m_enabled = true;
      m_controller.reset(getMeasurement());
      DataLogManager.log("Arm Enabled");
    }
  }

  @Override
  /** Disables the PID control. Sets output to zero. */
  public void disable() {

    // Set goal to current position to minimize movement on re-enable and reset output
    m_enabled = false;
    setGoal(getMeasurement()); 
    useOutput(0, new State());
    DataLogManager.log("Arm Disabled");

  }

  /** Sets the goal state for the subsystem. Goal velocity assumed to be zero. */
  @Override
    public void setGoal(double goal) {
    setGoal(new TrapezoidProfile.State(goal, 0));
    goalPosition = goal;

  }

  private void setupShuffleboard() {

    SmartDashboard.putData(m_controller);

  }

  public void updateShuffleboard() {

    SmartDashboard.putBoolean("Arm Enabled", m_enabled);
    SmartDashboard.putNumber("Arm Goal", Units.radiansToDegrees(goalPosition));
    SmartDashboard.putNumber("Measured Angle", Units.radiansToDegrees(getMeasurement()));
    SmartDashboard.putNumber("Arm Velocity", Units.radiansToDegrees(getVelocity()));
    SmartDashboard.putNumber("Motor Voltage", voltageCommand); 
    SmartDashboard.putNumber("Battery Voltage",RobotController.getBatteryVoltage()); // sim
    SmartDashboard.putNumber("Motor Current", getCurrent()); 

  }

  @Override
  public void close() {
    motor.close();
  }
}
