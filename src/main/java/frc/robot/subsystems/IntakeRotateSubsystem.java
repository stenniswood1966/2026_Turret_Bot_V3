// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRotateSubsystem extends SubsystemBase {
  private Boolean killFlag = false;

  private static TalonFX motor1 = new TalonFX(constants.kg_IntakeRotateSubsystem.k_Motor1ID,constants.kg_IntakeRotateSubsystem.k_Motor1CANBus);
  private MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> motor1MotorPosition= motor1.getPosition(false);

  private final DutyCycleOut calibrationRequest = new DutyCycleOut(constants.kg_IntakeRotateSubsystem.k_HomePower)
    .withIgnoreHardwareLimits(true)
    .withIgnoreSoftwareLimits(true);

  /** Creates a new IntakeRotateSubsystem. */
  public IntakeRotateSubsystem() {
    var fx_cfg = new TalonFXConfiguration();

    //configuration for motor1
    fx_cfg.MotorOutput.Inverted = constants.kg_IntakeRotateSubsystem.k_Motor1InvertedValue;
    fx_cfg.MotorOutput.NeutralMode = constants.kg_IntakeRotateSubsystem.k_Motor1NeutralModeValue;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = constants.kg_IntakeRotateSubsystem.k_Motor1ForwardSoftLimitEnable;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = constants.kg_IntakeRotateSubsystem.k_Motor1ReverseSoftLimitEnable;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.kg_IntakeRotateSubsystem.k_Motor1ForwardSoftLimitThreshold;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =constants.kg_IntakeRotateSubsystem.k_Motor1ReverseSoftLimitThreshold;

    //amp limits
    fx_cfg.CurrentLimits.StatorCurrentLimit = constants.kg_IntakeRotateSubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = constants.kg_IntakeRotateSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = constants.kg_IntakeRotateSubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_IntakeRotateSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_IntakeRotateSubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLowerTime = constants.kg_IntakeRotateSubsystem.k_Motor1SupplyCurrentLowerTime;

    // set slot 0 gains
    var slot0Configs = fx_cfg.Slot0;
    slot0Configs.kS = constants.kg_IntakeRotateSubsystem.k_Motor1kS; 
    slot0Configs.kV = constants.kg_IntakeRotateSubsystem.k_Motor1kV;
    slot0Configs.kA = constants.kg_IntakeRotateSubsystem.k_Motor1kA;
    slot0Configs.kP = constants.kg_IntakeRotateSubsystem.k_Motor1kP;
    slot0Configs.kI = constants.kg_IntakeRotateSubsystem.k_Motor1kI;
    slot0Configs.kD = constants.kg_IntakeRotateSubsystem.k_Motor1kD;
    slot0Configs.GravityType = constants.kg_IntakeRotateSubsystem.k_Motor1GravityTypeValue;

    // set Motion Magic Settings
    var motionMagicConfigs = fx_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = constants.kg_IntakeRotateSubsystem.k_Motor1MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = constants.kg_IntakeRotateSubsystem.k_Motor1MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = constants.kg_IntakeRotateSubsystem.k_Motor1MotionMagicJerk;

    motor1.getConfigurator().apply(fx_cfg,0.050);
    motor1.setPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BaseStatusSignal.refreshAll(
      motor1MotorPosition
    );
  }

  private void enablemotionmagic(double targetpos) {
    if (!killFlag) {
      motor1.setControl(mmReq.withPosition(targetpos).withSlot(0));
    }
  }

  public Command stopCommand(){
    return run(() -> stop());
  }

  private void stop() {
    motor1.set(0);
  }
  
  public Trigger t_intakeIsOut(Angle threshold){
    return new Trigger(() -> {
      return motor1MotorPosition.isNear(Angle.ofRelativeUnits(constants.kg_IntakeRotateSubsystem.k_OutPosition, Rotation), threshold);
    });
  }

  public Trigger t_intakeIsIn(Angle threshold){
    return new Trigger(() -> {
      return motor1MotorPosition.isNear(Angle.ofRelativeUnits(constants.kg_IntakeRotateSubsystem.k_InPosition, Rotation), threshold);
    });
  }

  public Command setOutCommand(){
    return runOnce(() ->  setOutMethod());
  }

  private void setOutMethod(){
    if (!killFlag) {
      enablemotionmagic(constants.kg_IntakeRotateSubsystem.k_OutPosition);
    }
  }

  public Command setInCommand(){
    return runOnce(() -> setInMethod()); 
  }

  private void setInMethod(){
    if (!killFlag) {
      enablemotionmagic(constants.kg_IntakeRotateSubsystem.k_InPosition);
    }
  }

  public Command killCommand(){
    return runOnce(() -> killMethod());
  }

  private void killMethod(){
    stop();
    killFlag = !killFlag;
  }

  public Command wiggleCommand(){
    return run(() -> wiggleMethod());
  }

  private void wiggleMethod(){
    //if motor is at wiggle position set out position
    //if motor is at out position set to wiggle position 
    //if motor is moving dont do anything
    
    if (motor1MotorPosition.isNear(Angle.ofRelativeUnits(constants.kg_IntakeRotateSubsystem.k_OutPosition, Rotation), constants.kg_IntakeRotateSubsystem.k_threshold)) {
      setWiggleMethod();
    } else if(motor1MotorPosition.isNear(Angle.ofRelativeUnits(constants.kg_IntakeRotateSubsystem.k_WigglePosition, Rotation), constants.kg_IntakeRotateSubsystem.k_threshold)) {
      setOutMethod();
    }
  }

  private void setWiggleMethod(){
    if (!killFlag) {
      enablemotionmagic(constants.kg_IntakeRotateSubsystem.k_WigglePosition);
    }
  }

  public final Trigger isHardStop = new Trigger(() -> {
    return motor1.getVelocity().getValue().abs(RotationsPerSecond) < 1 &&
      motor1.getTorqueCurrent().getValue().abs(Amps) > 30;
  }).debounce(0.1);

  public Command homeCommand(){
    return run(() -> {
      motor1.setControl(calibrationRequest);
    })
    .until(isHardStop)
    .andThen(stopCommand()
    .finallyDo(() -> {
      motor1.setPosition(constants.kg_IntakeRotateSubsystem.k_Motor1ForwardSoftLimitThreshold);
    }));
  }
}
