// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public static final class MotorConstants{
        public static final int m_intakeMotor = 11;
        public static final int m_pivotMotor = 12;
        public static final int m_transitionMotor = 13;
        public static final int m_shooterTopMotor = 14;
        public static final int m_shooterBottomMotor = 15;
        public static final int m_climberMotor = 16;

        public static final String m_CANbus = "CANivoreV1";
    }

    public static final class OIConstants{
        public static final int m_driverControllerPort = 1;
        public static final int m_operatorControllerPort = 0;
    }

    public static final class SpeedConstants{
        public static final double kIntakeSpeed = .85;
        public static final double kPivotSpeed = 0.4;
        public static final double kTransitionSpeed = 0.5;
        public static final double kShootTopAmp = 0.5;
        public static final double kShootBottomAmp = -0.5;
        public static final double kShootSubwoofer = 0.5;  
        public static final double kShootPodium = 0.5;
        public static final double kNoShoot = 0.3;
    }
}
