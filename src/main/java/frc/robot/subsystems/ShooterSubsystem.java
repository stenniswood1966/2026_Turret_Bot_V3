// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
  private static TalonFX motor1 = new TalonFX (constants.kg_ShooterSubsystem.k_Motor1ID, constants.kg_ShooterSubsystem.k_Motor1CANBus);
  private static TalonFX motor2 = new TalonFX(constants.kg_ShooterSubsystem.k_Motor2ID, constants.kg_ShooterSubsystem.k_Motor2CANBus);

  private final VelocityDutyCycle m_velocityTorque = new VelocityDutyCycle(0).withSlot(0);

  private final StatusSignal<AngularVelocity> motor1MotorVelocity = motor1.getVelocity(false);

  private boolean killFlag = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //configure the TalonFX motors
    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration

    //configuration for motor1
    fx_cfg.MotorOutput.Inverted = constants.kg_ShooterSubsystem.k_Motor1InvertedValue;
    fx_cfg.MotorOutput.NeutralMode = constants.kg_ShooterSubsystem.k_Motor1NeutralModeValue;

    //amp limits
    fx_cfg.CurrentLimits.StatorCurrentLimit = constants.kg_ShooterSubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = constants.kg_ShooterSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = constants.kg_ShooterSubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_ShooterSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_ShooterSubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLowerTime = constants.kg_ShooterSubsystem.k_Motor1SupplyCurrentLowerTime;

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    fx_cfg.Slot0.kS = constants.kg_ShooterSubsystem.k_Motor1kS; // To account for friction, add 2.5 A of static feedforward
    fx_cfg.Slot0.kP = constants.kg_ShooterSubsystem.k_Motor1kP; // An error of 1 rotation per second results in 5 A output
    fx_cfg.Slot0.kI = constants.kg_ShooterSubsystem.k_Motor1kI; // No output for integrated error
    fx_cfg.Slot0.kD = constants.kg_ShooterSubsystem.k_Motor1kD; // No output for error derivative
    fx_cfg.Slot0.kV = constants.kg_ShooterSubsystem.k_Motor1kV;
    fx_cfg.Slot0.kA = constants.kg_ShooterSubsystem.k_Motor1KA;

    fx_cfg.MotorOutput.PeakForwardDutyCycle = constants.kg_ShooterSubsystem.k_Motor1PeakForwardDutyCycle;
    fx_cfg.MotorOutput.PeakReverseDutyCycle = constants.kg_ShooterSubsystem.k_Motor1PeakReverseDutyCycle;

    // Peak output of 40 A
    fx_cfg.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(constants.kg_ShooterSubsystem.k_Motor1PeakForwardTorqueCurrent))
      .withPeakReverseTorqueCurrent(Amps.of(constants.kg_ShooterSubsystem.k_Motor1PeakReverseTorqueCurrent));

    motor1.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor

    //config motor 2
    var fx_cfg2 = new TalonFXConfiguration(); //creates a default TalonFX configuration

    fx_cfg2.CurrentLimits.StatorCurrentLimit = constants.kg_ShooterSubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg2.CurrentLimits.StatorCurrentLimitEnable = constants.kg_ShooterSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg2.CurrentLimits.SupplyCurrentLimit = constants.kg_ShooterSubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg2.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_ShooterSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg2.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_ShooterSubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg2.CurrentLimits.SupplyCurrentLowerTime = constants.kg_ShooterSubsystem.k_Motor1SupplyCurrentLowerTime;

    //configuration for motor 2
    fx_cfg2.MotorOutput.NeutralMode = constants.kg_ShooterSubsystem.k_Motor2NeutralModeValue;

    motor2.getConfigurator().apply(fx_cfg2, 0.050);

    motor2.setControl(new Follower(constants.kg_ShooterSubsystem.k_Motor1ID, constants.kg_ShooterSubsystem.k_Motor2AlignmentValue));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler runOnce
    BaseStatusSignal.refreshAll(
      motor1MotorVelocity
    );
  }

  public Command stopCommand(){
    return runOnce(() -> stop());
  }

  private void stop()
  {
    motor1.set(0);
  }

  public AngularVelocity getMotor1Velocity(){
    return motor1MotorVelocity.getValue();
  }

  public Trigger t_shooterInRange(AngularVelocity threshold){
    return new Trigger(() -> {
      return motor1MotorVelocity.isNear(RotationsPerSecond.of(m_velocityTorque.Velocity), threshold);
    });
  }

  public Command setTarget(Supplier<Double> target){
    return runOnce(() -> setTargetMethod(target));
  }

  public void setTargetMethod(Supplier<Double> target){
    if (!killFlag) {
      double t = target.get();
      motor1.setControl(m_velocityTorque.withVelocity(Math.min(t, constants.kg_ShooterSubsystem.k_Motor1MaxRPS)));
    }
  }

  public Command killCommand(){
    return runOnce(() -> killMethod());
  }

  private void killMethod(){
    stop();
    killFlag = !killFlag;
  }
}
