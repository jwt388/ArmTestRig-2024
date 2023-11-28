// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
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
// added for simulation
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
  private double lastPosition = 0.0;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor armGearbox = DCMotor.getVex775Pro(2);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          Constants.kArmReduction,
          SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
          Constants.kArmLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          Constants.kStartAngleRads,
          VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );

  private double encoderSimDistance;
  private double encoderSimRate;
  private double simCurrent;


  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(70, 60);
  private final MechanismRoot2d mechArmPivot = mech2d.getRoot("ArmPivot", 25, 30);
  private final MechanismLigament2d mechArmTower =
      mechArmPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d mechArm =
      mechArmPivot.append(
          new MechanismLigament2d(
              "Arm",
              Constants.kArmLengthInches,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

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

    if (RobotBase.isSimulation()) {
      simulationInit();
    }

    setupShuffleboard();

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", mech2d);
    mechArmTower.setColor(new Color8Bit(Color.kBlue));

  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
    }
    updateShuffleboard();

  }

  public void simulationInit() {

    encoderSimDistance = 0;

    // This shouldn't be needed in 2024 since SingleJointedArmSim will allow setting in constructor
    armSim.setState(ArmConstants.kArmOffsetRads,0);
  }

  /** Update the simulation model. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(voltageCommand);

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    double newPosition = armSim.getAngleRads() - ArmConstants.kArmOffsetRads;
    encoderSimDistance = newPosition;
    encoderSimRate = (newPosition-lastPosition)/0.02;
    lastPosition = newPosition;

    // SimBattery estimates loaded battery voltages
    simCurrent = armSim.getCurrentDrawAmps();
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simCurrent));

    // Update the Mechanism Arm angle based on the simulated arm angle
    mechArm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double newFeedforward = 0;
    if (m_enabled) {
      // Calculate the feedforward from the sepoint
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

  /** Extend the arm - only affects simulation display for now - no affect on MOI*/
  public void extendArm() {
    mechArm.setLength(Constants.kArmExtendedLengthInches);
  }

  /** Retract the arm - only affects simulation display for now - no affect on MOI*/
  public void retractArm() {
    mechArm.setLength(Constants.kArmLengthInches);
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
    SmartDashboard.putNumber("Current", getCurrent()); 

    if (RobotBase.isSimulation()) {
      SmartDashboard.putNumber("Mechanical Angle", Units.radiansToDegrees(armSim.getAngleRads())); //sim
    }

  }

  @Override
  public void close() {
    motor.close();
    mech2d.close();
    mechArmPivot.close();
    mechArm.close();
  }
}
