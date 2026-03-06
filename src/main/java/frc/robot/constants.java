// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;


/** Add your docs here. */
public class constants {

    //this holds the turret offsets
    public class kg_TurretOffset{
        public static Pose2d k_TurretOffsetPose2d = new Pose2d(Units.inchesToMeters(-5.5),Units.inchesToMeters(3.5), null);
    }

    //target locations and field stuff
    public class kg_TargetsAndField{
        //field size
        public static final AprilTagFieldLayout FIELD_LAYOUT;
        static {
            FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }

        //Field dimensions
        public static final double k_FieldLength = FIELD_LAYOUT.getFieldLength();
        public static final double k_FieldWidth = FIELD_LAYOUT.getFieldWidth();

        //Zones
        public static double k_RedAllianceZone = k_FieldLength - Units.inchesToMeters(156.61) - Units.inchesToMeters(48);
        public static double k_BlueAllianceZone = Units.inchesToMeters(156.61) + Units.inchesToMeters(48);
        public static double k_MiddleOfField = k_FieldWidth/2;

        //red targets
        public static Pose2d k_HubPose2dRed = new Pose2d(FIELD_LAYOUT.getTagPose(10).get().getX() - Units.inchesToMeters(47/2) ,k_MiddleOfField,null);
        public static Pose2d k_PassLeftPose2dRed = new Pose2d(k_FieldLength - 1,1,null);
        public static Pose2d k_PassRightPose2dRed = new Pose2d(k_FieldLength - 1,k_FieldWidth - 1,null);

        //blue targets
        public static Pose2d k_HubPose2dBlue = new Pose2d(FIELD_LAYOUT.getTagPose(26).get().getX() + Units.inchesToMeters(47/2),k_MiddleOfField,null);
        public static Pose2d k_PassLeftPose2dBlue = new Pose2d(1,k_FieldWidth - 1,null);
        public static Pose2d k_PassRightPose2dBlue = new Pose2d(1,1,null);
    }

    //Settings for the ClimberSubsystem
    public class kg_ClimberSubsystem{
        //Motor(s) Configs
        public static int k_Motor1ID = 13;
        public static CANBus k_Motor1CANBus = new CANBus("Flurb_Drive");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.Clockwise_Positive;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Brake;
        public static Boolean k_Motor1ForwardSoftLimitEnable = true;
        public static Boolean k_Motor1ReverseSoftLimitEnable = true;
        public static double k_Motor1ForwardSoftLimitThreshold = 98.021973;
        public static double k_Motor1ReverseSoftLimitThreshold = 0;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLimit = 70;
        public static boolean k_Motor1SupplyCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLowerLimit = 70;
        public static double k_Motor1SupplyCurrentLowerTime = 0;

        //set slot 0 gains
        public static double k_Motor1kS = 0; 
        public static double k_Motor1kV = 0;
        public static double k_Motor1kA = 0;
        public static double k_Motor1kP = 10;//use 10
        public static double k_Motor1kI = 0;
        public static double k_Motor1kD = 0;
        public static GravityTypeValue k_Motor1GravityTypeValue = GravityTypeValue.Elevator_Static;

        //set Motion Magic Settings
        public static double k_Motor1MotionMagicCruiseVelocity = 90;
        public static double k_Motor1MotionMagicAcceleration = 90;
        public static double k_Motor1MotionMagicJerk = 1600;

        //Motion Magic Setpoints
        public static double k_UpPosition = 98;
        public static double k_DownPosition = 15;

        //Power setting
        public static double k_HomePower = -0.3;
    }

    //Settings for the FeederSubsystem
    public class kg_FeederSubsystem{
        //Motor(s) Configs
        public static int k_Motor1ID = 14;
        public static CANBus k_Motor1CANBus = new CANBus("Flurb_Drive");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.CounterClockwise_Positive;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Coast;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLimit = 70;
        public static boolean k_Motor1SupplyCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLowerLimit = 70;
        public static double k_Motor1SupplyCurrentLowerTime = 0;
        
        //Power Settings
        public static double k_FeedPower = 1;
        public static double k_AntiJamPower = -1;

        //thresholds/setpoints
        public static AngularVelocity k_FeederOnThreshold = RotationsPerSecond.of(30);//2.5
    }

    //Settings for the HoodSubsystem
    public class kg_HoodSubsystem{
        //Motor(s) Configs
        public static int k_Motor1ID = 10;
        public static CANBus k_Motor1CANBus = new CANBus("Flurb_Drive");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.Clockwise_Positive;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Brake;
        public static Boolean k_Motor1ForwardSoftLimitEnable = true;
        public static Boolean k_Motor1ReverseSoftLimitEnable = true;
        public static double k_Motor1ForwardSoftLimitThreshold = 7.49;//was 17
        public static double k_Motor1ReverseSoftLimitThreshold = 0;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLimit = 120;
        public static boolean k_Motor1SupplyCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLowerLimit = 120;
        public static double k_Motor1SupplyCurrentLowerTime = 0;
        
        //set slot 0 gains
        public static double k_Motor1kS = 0.35;//was.35 
        public static double k_Motor1kV = 0.0;
        public static double k_Motor1kA = 0;
        public static double k_Motor1kP = 5;//was 10
        public static double k_Motor1kI = 0;
        public static double k_Motor1kD = 0;
        public static StaticFeedforwardSignValue k_Motor1StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        public static GravityTypeValue k_Motor1GravityTypeValue = GravityTypeValue.Elevator_Static;

        //set Motion Magic Settings
        public static double k_Motor1MotionMagicCruiseVelocity = 254;//was 15
        public static double k_Motor1MotionMagicAcceleration = 254;//was 15
        public static double k_Motor1MotionMagicJerk = 1600;

        //Motion Magic Setpoints
        public static double k_HomePosition = 0;

        //threshold
        public static Angle k_threshold = Angle.ofBaseUnits(.5, Revolution);

        //Power setting
        public static double k_HomePower = -0.3;
    }

    //Settings for the IntakeRotateSubsystem
    public class kg_IntakeRotateSubsystem{
        //Motor(s) Configs
        public static int k_Motor1ID = 6;
        public static CANBus k_Motor1CANBus = new CANBus("rio");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.CounterClockwise_Positive;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Brake;
        public static Boolean k_Motor1ForwardSoftLimitEnable = true;
        public static Boolean k_Motor1ReverseSoftLimitEnable = true;
        public static double k_Motor1ForwardSoftLimitThreshold = 15.931641;
        public static double k_Motor1ReverseSoftLimitThreshold = 0;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLimit = 70;
        public static boolean k_Motor1SupplyCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLowerLimit = 70;
        public static double k_Motor1SupplyCurrentLowerTime = 0;
        
        //set slot 0 gains
        public static double k_Motor1kS = 0; 
        public static double k_Motor1kV = 0;
        public static double k_Motor1kA = 0;
        public static double k_Motor1kP = 50;
        public static double k_Motor1kI = 0;
        public static double k_Motor1kD = 0;
        public static GravityTypeValue k_Motor1GravityTypeValue = GravityTypeValue.Arm_Cosine;

        //set Motion Magic Settings
        public static double k_Motor1MotionMagicCruiseVelocity = 30;
        public static double k_Motor1MotionMagicAcceleration = 30;//was 15
        public static double k_Motor1MotionMagicJerk = 1600;

        //Motion Magic Setpoints
        public static double k_InPosition = 0;
        public static double k_OutPosition = 15.9;
        public static double k_WigglePosition = 8;

        //threshold
        public static Angle k_threshold = Angle.ofBaseUnits(1, Revolution);

        //Power setting
        public static double k_HomePower = 0.3;
    }

    //Settings for the IntakeSubsystem
    public class kg_IntakeSubsystem{
        //Motor(s) Configs
        public static int k_Motor1ID = 5;
        public static CANBus k_Motor1CANBus = new CANBus("rio");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.Clockwise_Positive;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Brake;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLimit = 70;
        public static boolean k_Motor1SupplyCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLowerLimit = 70;
        public static double k_Motor1SupplyCurrentLowerTime = 0;
        
        //Power Settings
        public static double k_IntakePower = .8;
        public static double k_AntiJamPower = -.75;
    }

    //Settings for the PASubsystem
    public class kg_PASubsystem {
        //Motor(s) Configs
        public static int k_Motor1ID = 15;
        public static CANBus k_Motor1CANBus = new CANBus("Flurb_Drive");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.CounterClockwise_Positive;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Coast;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLimit = 70;
        public static boolean k_Motor1SupplyCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLowerLimit = 70;
        public static double k_Motor1SupplyCurrentLowerTime = 0;
            
        //Power Settings
        public static double k_FeedPower = 1;
        public static double k_AntiJamPower = -1;

        //Thresholds
        public static AngularVelocity k_StopThreshold = RotationsPerSecond.of(1);//2.5
    }

    //Setting for the ShooterSubsystem
    public class kg_ShooterSubsystem{
        //Motor(s) Configs
        public static int k_Motor1ID = 11;
        public static int k_Motor2ID = 12;
        public static CANBus k_Motor1CANBus = new CANBus("Flurb_Drive");
        public static CANBus k_Motor2CANBus = new CANBus("Flurb_Drive");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.Clockwise_Positive;
        public static MotorAlignmentValue k_Motor2AlignmentValue = MotorAlignmentValue.Opposed;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Coast;
        public static NeutralModeValue k_Motor2NeutralModeValue = NeutralModeValue.Coast;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = false;
        public static double k_Motor1SupplyCurrentLimit = 70;
        public static boolean k_Motor1SupplyCurrentLimitEnable = false;
        public static double k_Motor1SupplyCurrentLowerLimit = 70;
        public static double k_Motor1SupplyCurrentLowerTime = 0;
        
        //set slot 0 gains
        public static double k_Motor1kS = .25; // To account for friction, add 2.5 A of static feedforward
        public static double k_Motor1kP = .05; // An error of 1 rotation per second results in 5 A output
        public static double k_Motor1kI = 0; // No output for integrated error
        public static double k_Motor1kD = 0; // No output for error derivative
        public static double k_Motor1kV = 0;
        public static double k_Motor1KA = 0;
        //Peak output of 40 A
        public static double k_Motor1PeakForwardTorqueCurrent = 40;
        public static double k_Motor1PeakReverseTorqueCurrent = 0;
        public static double k_Motor1PeakForwardDutyCycle = 1;
        public static double k_Motor1PeakReverseDutyCycle = 0;


        //Thresholds
        public static AngularVelocity k_SpinUpThreshold = RotationsPerSecond.of(3);//2.5

        //Max rps
        public static double k_Motor1MaxRPS = 40;
    }   

    //Setting for the TurretSubsystem
    public class kg_TurretSubsystem{
        //Motor(s) Configs
        public static int k_Motor1ID = 16;
        public static CANBus k_Motor1CANBus = new CANBus("Flurb_Drive");
        public static InvertedValue k_Motor1InvertedValue = InvertedValue.Clockwise_Positive;
        public static NeutralModeValue k_Motor1NeutralModeValue = NeutralModeValue.Brake;
        public static Boolean k_Motor1ForwardSoftLimitEnable = true;
        public static Boolean k_Motor1ReverseSoftLimitEnable = true;
        public static double k_Motor1ForwardSoftLimitThreshold = 18.758301;
        public static double k_Motor1ReverseSoftLimitThreshold = -20.439453;

        //amp limits
        public static double k_Motor1StatorCurrentLimit = 120;
        public static boolean k_Motor1StatorCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLimit = 70;
        public static boolean k_Motor1SupplyCurrentLimitEnable = true;
        public static double k_Motor1SupplyCurrentLowerLimit = 70;
        public static double k_Motor1SupplyCurrentLowerTime = 0;
        
        //set slot 0 gains
        public static double k_Motor1kS = 0.05; 
        public static double k_Motor1kV = 0;
        public static double k_Motor1kA = 0;
        public static double k_Motor1kP = 20;//40
        public static double k_Motor1kI = 0;
        public static double k_Motor1kD = 0;
        public static StaticFeedforwardSignValue k_Motor1StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        public static GravityTypeValue k_Motor1GravityTypeValue = GravityTypeValue.Elevator_Static;

        //set Motion Magic Settings
        public static double k_Motor1MotionMagicCruiseVelocity = 3000;
        public static double k_Motor1MotionMagicAcceleration = 3000;
        public static double k_Motor1MotionMagicJerk = 9000;

        //Motion Magic Setpoints
        public static double k_HomePosition = 0;

        //threshold
        public static Angle k_threshold = Angle.ofBaseUnits(.5, Rotation);
    }
}