// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class ArmSimulation extends SubsystemBase implements AutoCloseable {
   
  private final ArmSubsystem armSubsystem;
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
  public ArmSimulation(ArmSubsystem simulationArmSubsystem) {

    armSubsystem = simulationArmSubsystem;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", mech2d);
    mechArmTower.setColor(new Color8Bit(Color.kBlue));

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
    armSim.setInput(armSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    double newPosition = armSim.getAngleRads() - ArmConstants.kArmOffsetRads;
    encoderSimDistance = newPosition;
    double encoderSimRate = (newPosition-lastPosition)/0.02;
    lastPosition = newPosition;

    // SimBattery estimates loaded battery voltages
    double simCurrent = armSim.getCurrentDrawAmps();
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simCurrent));

    // Update the Mechanism Arm angle based on the simulated arm angle
    mechArm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    // Update the arm subsystem to follow the simulation
    armSubsystem.setSimDistance(encoderSimDistance);
    armSubsystem.setSimRate(encoderSimRate);
    armSubsystem.setSimCurrent(simCurrent);

    updateShuffleboard();

  }
  
  public void updateShuffleboard() {

    SmartDashboard.putNumber("Arm Mechanical Angle", Units.radiansToDegrees(armSim.getAngleRads())); //sim

  }

  @Override
  public void close() {
    mech2d.close();
    mechArmPivot.close();
    mechArm.close();
  }
}

