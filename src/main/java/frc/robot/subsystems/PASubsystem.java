// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants;

public class PASubsystem extends SubsystemBase {

  private Boolean killFlag = false;
  private static TalonFX motor1 = new TalonFX(constants.kg_PASubsystem.k_Motor1ID, constants.kg_PASubsystem.k_Motor1CANBus);//change me
  
  private final StatusSignal<AngularVelocity> motor1MotorVelocity = motor1.getVelocity(false);

  /** Creates a new PASubsystem. */
  public PASubsystem() {
    //configure the TalonFX motors
    var fx_cfg = new TalonFXConfiguration();// create a default TalonFX configuration
    //configuration for motor1
    fx_cfg.MotorOutput.Inverted = constants.kg_PASubsystem.k_Motor1InvertedValue;
    fx_cfg.MotorOutput.NeutralMode = constants.kg_PASubsystem.k_Motor1NeutralModeValue;
    fx_cfg.Voltage.PeakForwardVoltage = constants.kg_PASubsystem.k_Motor1PeakForwardVoltage;
    fx_cfg.Voltage.PeakReverseVoltage = constants.kg_PASubsystem.k_Motor1PeakReverseVoltage;

    //amp limits
    fx_cfg.CurrentLimits.StatorCurrentLimit = constants.kg_PASubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = constants.kg_PASubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = constants.kg_PASubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_PASubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_PASubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLowerTime = constants.kg_PASubsystem.k_Motor1SupplyCurrentLowerTime;

    motor1.getConfigurator().apply(fx_cfg, 0.05); //apply configuration to motor
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

  public Command feedCommand(){
    return runOnce(() -> feedMethod());
  }

  private void feedMethod(){
    if (!killFlag) {
      motor1.setControl(new DutyCycleOut(constants.kg_PASubsystem.k_FeedPower));
    }
  }

  public Command antiJamCommand(){
    return runOnce(() -> antiJamMethod());
  }

  private void antiJamMethod(){
    if (!killFlag) {
      motor1.setControl(new DutyCycleOut(constants.kg_PASubsystem.k_AntiJamPower));
    }
  }

  public Command killCommand(){
    return runOnce(() -> killMethod());
  }

  private void killMethod(){
    stop();
    killFlag = !killFlag;
  }

  public Trigger t_paStopped(AngularVelocity threshold){
    return new Trigger(() -> {
      return motor1MotorVelocity.getValue().lt(threshold);
    });
  }
}
