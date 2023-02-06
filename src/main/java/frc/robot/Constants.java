// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final class GlobalConstants {
        public static final double kLoopTime = 0.020;
        public static final float INF = (float)Math.pow(10, 5); // This represents the Infinite
    }

    public static final class DriveConstants {
        public static final double kMaxAcceleration = 3.0;
        public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly
        public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly

        public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                          // read when not in use (Eliminates "Stick Drift")
        public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                          // when aimed at a 45deg angle (Such that X and Y are are
                                                          // maximized simultaneously)
        public static final double kTranslationSlew = 1.45;
        public static final double kRotationSlew = 3.00;
    }

    public static final class FieldConstants{
        public static final double OdometryToFieldOffsetX= 8.23;
        public static final double OdometryTOFieldOffsety=4.01;
    }
    public static final class SemiAutoConstants{
        public static final double kSemiAutoVelocityP=0.10 ;//TODO
        public static final double kSemiAutoVelocityI=0.00;//TODO
        public static final double kSemiAutoVelocityD=0.05;//TODO
        public static final Constraints kSemiAutoVelocityConstrants =new Constraints(1,1);//TODO
        
        public static final double kSemiAutoOmegaP=3;//TODO
        public static final double kSemiAutoOmegaI=0.0;//TODO
        public static final double kSemiAutoOmegaD=0.0;//TODO
        public static final Constraints kSemiAutoOmegaConstrants =new Constraints(1,100);//TODO
        public static final double SemiAutoOmegaSlewRate=2;
        public static final double SemiAutoOmegaMax=2;
    }
    
    public static final class SwerveConstants{
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = 29/15*60/15;
        public static final double angleGearRatio = 56/6*60/10; 
        
        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.69552 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.8378 / 12);
        public static final double driveKA = (0.44473 / 12);

        // Pigeon Port
        public static final int PigeonIMUPort = 17;

        public static double kDriveMotorMaxOutput = 1;
        public static double kPivotMotorMaxOutput = 1;
    
        public static double kDriveMotorNeutralDeadband = 0;
        public static double kPivotMotorNeutralDeadband = 0;
        
        public static double VelocityCorrectionFactor = 0.5;//TODO
        /**
         * The first version of PID parameters
         *                         Value
         * @param kDriveMotorkP    0.025
         * @param kDriveMotorkI    0.0016
         * @param kDriveMotorkD    2.5
         * @param kDriveMotorkF    0.06
         * @param kDriveMotorIZone 240
         * 
         * 车子在行驶过程中基本不抖动，底盘PID大部分情况下是正常的。
         * 
         */
        //以下参数都被用在swerve drive train中，用来定义电机。
        public static double kDriveMotorkP = 0.1; // 5e-2 0.05   0.025  swervemodule 动轮模块P数值
        public static double kDriveMotorkI = 0; //5e-4 0.005  0.0016    swervemodule 动轮模块I数值
        public static double kDriveMotorkD = 0; //   5e-0 5 1.5  2.5    swervemodule 动轮模块D数值
        public static double kDriveMotorkF = 0.042;//   0.045       0.06swervemodule 动轮模块F数值
        public static double kDriveMotorIZone = 0;// 90          240    swervemodule 动轮模块I的积分上限

        public static double kPivotMotorkP = 3;//3  swervemodule 转向轮P数值
        public static double kPivotMotorkI = 0;//   swervemodule 转向轮I数值
        public static double kPivotMotorkD = 100;// swervemodule 转向轮D数值
        public static double kPivotMotorF = 0;//    swervemodule 转向轮F数值
        public static double kPivotMotorkIZone = 0;//swervemodule转向轮I的积分上限
        public static double motionCruiseVelocity = 1200;//swervemodule转向轮motion magic的最大速度，单位是unit/100ms
        public static double motionAcceleration = 3500; //swervemodule 转向轮motion magic的加速度，单位是unit/(100ms*s)
    
        public static double kLoopSeconds = 0.0;//我也不知道干什么用的参数，晚些时候问问zwb
    
        public static double kDriveEncoderResolution = 2048.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad swervemodule 动轮模块编码器分辨率，用来换算用的，单位是unit/2rad
        public static double kPivotEncoderResolution = 4096.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad swervemodule 转向轮模块编码器分辨率，用来换算用的，单位是unit/2rad
    
        public static double kDriveMotorReductionRatio = 1.0 / (29/15*60/15); //29/15*60/15 动轮模块的减速比
        public static double kPivotMotorReductionRatio = 1.0 / (56/6*60/10); //56/6*60/10   转向模块的减速比
        public static double kDriveEncoderReductionRatio = 1.0 / (29 / 15 * 60 / 15);//     动轮模块的编码器的减速比 即最后输出的角度:编码器实际上读到的数据
        public static double kPivotEncoderReductionRatio = -1.0 / 1.0;//                    转向模块的编码器的减速比 即最后输出的角度:编码器实际上读到的数据
    
        public static double kWidth  = 0.572936;//The unit is 0.342_m 左右两个drivemodule之间的间距
        public static double kLength = 0.572936;//The unit is 0.342_m 前后两个drivemodule之间的间距 我也不知道为什么单位是0.342米
    
        public static double kWheelDiameter = 0.093;//轮子直径 单位是米
    
        public static double kMaxSpeed = 4;//The unit is meters per second 轮子的最大转速，单位是m/s
    
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(//创建一个坐标系，上面的每一个点就是底盘上的一个drivemodule
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, -kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, -kWidth / 2.0));
    
        public static final double wheelCircumference = kWheelDiameter * Math.PI;//轮子周长

        public static final SwerveModulePosition[] m_InitialSwerveModulePositions={new SwerveModulePosition()};

        public static final double AprilTagLockTime =0.05;//apriltag在视野里出现多少秒就重新校准odometry
    }
}
