// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimberSubsystem extends SubsystemBase {

  private static TalonFX motor1 = new TalonFX(constants.kg_ClimberSubsystem.k_Motor1ID,constants.kg_ClimberSubsystem.k_Motor1CANBus);
  MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  private final DutyCycleOut calibrationRequest = new DutyCycleOut(constants.kg_ClimberSubsystem.k_HomePower)
    .withIgnoreHardwareLimits(true)
    .withIgnoreSoftwareLimits(true);

  /** Creates a new IntakeRotateSubsystem. */
  public ClimberSubsystem() {
    var fx_cfg = new TalonFXConfiguration();

    //configuration for motor1
    fx_cfg.MotorOutput.Inverted = constants.kg_ClimberSubsystem.k_Motor1InvertedValue;
    fx_cfg.MotorOutput.NeutralMode = constants.kg_ClimberSubsystem.k_Motor1NeutralModeValue;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants.kg_ClimberSubsystem.k_Motor1ForwardSoftLimitEnable;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants.kg_ClimberSubsystem.k_Motor1ReverseSoftLimitEnable;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.kg_ClimberSubsystem.k_Motor1ForwardSoftLimitThreshold;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =constants.kg_ClimberSubsystem.k_Motor1ReverseSoftLimitThreshold;

    //amp limits
    fx_cfg.CurrentLimits.StatorCurrentLimit = constants.kg_ClimberSubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = constants.kg_ClimberSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = constants.kg_ClimberSubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_ClimberSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_ClimberSubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLowerTime = constants.kg_ClimberSubsystem.k_Motor1SupplyCurrentLowerTime;

    // set slot 0 gains
    var slot0Configs = fx_cfg.Slot0;
    slot0Configs.kS = constants.kg_ClimberSubsystem.k_Motor1kS; 
    slot0Configs.kV = constants.kg_ClimberSubsystem.k_Motor1kV;
    slot0Configs.kA = constants.kg_ClimberSubsystem.k_Motor1kA;
    slot0Configs.kP = constants.kg_ClimberSubsystem.k_Motor1kP;
    slot0Configs.kI = constants.kg_ClimberSubsystem.k_Motor1kI;
    slot0Configs.kD = constants.kg_ClimberSubsystem.k_Motor1kD;
    slot0Configs.GravityType = constants.kg_ClimberSubsystem.k_Motor1GravityTypeValue;

    // set Motion Magic Settings
    var motionMagicConfigs = fx_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = constants.kg_ClimberSubsystem.k_Motor1MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = constants.kg_ClimberSubsystem.k_Motor1MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = constants.kg_ClimberSubsystem.k_Motor1MotionMagicJerk;

    motor1.getConfigurator().apply(fx_cfg,0.050);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler runOnce
  }

  private void enablemotionmagic(double targetpos) {
    motor1.setControl(mmReq.withPosition(targetpos).withSlot(0));
  }

  private void stop() {
    motor1.set(0);
  }

  public double getPosition() {
    return Math.abs(motor1.getPosition().getValueAsDouble());
  }

  public Command setUpPositionCommand(){
    return runOnce(() -> enablemotionmagic(constants.kg_ClimberSubsystem.k_UpPosition));
  }

  public Command setDownPositionCommand(){
    return runOnce(() -> enablemotionmagic(constants.kg_ClimberSubsystem.k_DownPosition));
  }

  public Command stopCommand(){
    return runOnce(() -> stop());
  }

  public final Trigger isHardStop = new Trigger(() -> {
    return motor1.getVelocity().getValue().abs(RotationsPerSecond) < 1 &&
      motor1.getTorqueCurrent().getValue().abs(Amps) > 10;
  }).debounce(0.1);

  public Command homeCommand(){
    return runOnce(() -> {
      motor1.setControl(calibrationRequest);
    })
    .until(isHardStop)
    .andThen(stopCommand()
    .finallyDo(() -> {
      motor1.setPosition(0);
    }));
  }
}
