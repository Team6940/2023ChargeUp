// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SemiAutoSwerveControll;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        public static final double kTranslationSlew = 4;
        public static final double kRotationSlew = 3.00;
        public static final double kGamePieceFocusPIDControllerP=2.8;//TODO
        public static final double kGamePieceFocusPIDControllerI=0;//TODO
        public static final double kGamePieceFocusPIDControllerD=0.2;//TODO
    }

    public static final class FieldConstants{
        public static final double OdometryToFieldOffsetX= 8.23;
        public static final double OdometryTOFieldOffsety=4.01;
    }
    public static final class SemiAutoConstants{
        public static final double kSemiAutoVelocityP=2 ;//TODO
        public static final double kSemiAutoVelocityI=0.00;//TODO
        public static final double kSemiAutoVelocityD=0.0;//TODO
        public static final Constraints kSemiAutoVelocityConstrants =new Constraints(2,1);//TODO
        public static final double SemiAutoVelocityMax=2;
        public static final double kSemiAutoOmegaP=3.5;//TODO
        public static final double kSemiAutoOmegaI=0.0;//TODO
        public static final double kSemiAutoOmegaD=0.0;//TODO
        public static final Constraints kSemiAutoOmegaConstrants =new Constraints(1,100);//TODO
        public static final double SemiAutoOmegaSlewRate=2;
        public static final double SemiAutoOmegaMax=2;
    }
    public static final class ClawConstants{
        public static final int m_ClawSolenoidPort=2;
    }
    public static final class ArmConstants{//TODO
        public static final int ArmMotorUpDeviceNumber=9;
        public static final int ArmMotorDownDeviceNumber=10;
        public static final double ArmInDegreeToVerticalOffset=-15;
        public static final double ArmOutDegreeToVerticalOffset=-15;
        public static final double ArmMotorInkP=0.0025;
        public static final double ArmMotorInkI=0.00055;
        public static final double ArmMotorInkD=0.0008;
        public static final double ArmMotorOutkP=0.0055;
        public static final double ArmMotorOutkI=0.00;
        public static final double ArmMotorOutkD=0.0008;
        public static final double kArmMotorReductionRatio=(1./10.)*(32./72.)*360./2048.;
        public static final double kArmMotorUpOffeset=0;
        public static final double kArmMotorDownOffeset=0;

        public static final double ArmInSlewRateLimit=4;
        public static final double ArmOutSlewRateLimit=4;
        public static final double ArmInFeedForwardkS=0;
        public static final double ArmInFeedForwardkV=0.7;
        public static final double ArmInFeedForwardkA=0;
        public static final double ArmInFeedForwardkG=-0.08;
        

        public static final double ArmOutFeedForwardkS=0;
        public static final double ArmOutFeedForwardkV=0.7;
        public static final double ArmOutFeedForwardkA=0;
        public static final double ArmOutFeedForwardkG=0.1;
        public static final int ArmSolenoidPort=1;
    }   
    public static final class LayDownPreventionConstants{
        public static final int DownPreventionPort=3;    
    }
    public static final class ClimbConstants{
        public static double ClimbPIDControllerkP=0.6;
        public static double ClimbPIDControllerkI=0;
        public static double ClimbPIDControllerkD=0.5;
    }
    public static final class GameConstants
    {
        public static double ChargeStationLeanDegreesWhenDown=15;
        public static double Areas[][]={{3.29,7.90,13.18,5.78},
                                        {3,5.78,13.50,4.04},
                                        {4.89,3.99,11.60,2.70},
                                        {4.89,2.70,11.60,1.52},
                                        {3,2.70,13.6,0},
                                        {0,8,3.34,5.74},
                                        {13.20,8,16.24,5.38}};
        public static Pose2d SemiAutoBackPoseRed[]={new Pose2d(12.44-8.23,4.60-4.01, new Rotation2d(Math.PI)),
                                                    new Pose2d(14.36-8.23,4.72-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(10.82-8.23,4.84-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(10.82-8.23,0.18-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.25-8.23,0.67-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(4.31-8.23,6.90-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(12.07-8.23,6.90-4.01,new Rotation2d(Math.PI)),};
        public static Pose2d SemiAutoBackPoseBlue[]={new Pose2d(4.09-8.23,4.78-4.01, new Rotation2d(0)),
                                                     new Pose2d(2.22-8.23,4.61-4.01,new Rotation2d(0)),
                                                    new Pose2d(5.59-8.23,4.78-4.01,new Rotation2d(0)),
                                                new Pose2d(5.51-8.23,0.18-4.01,new Rotation2d(0)),
                                                new Pose2d(2.10-8.23,0.67-4.01,new Rotation2d(0)),
                                                new Pose2d(12.07-8.23,6.90-4.01,new Rotation2d(0)),
                                                new Pose2d(4.31-8.23,6.90-4.01,new Rotation2d(0)),
                                                new Pose2d(12.07-8.23,6.90-4.01,new Rotation2d(0)),};
        public static Pose2d SemiAutoGoPoseRed[]={new Pose2d(0.60-FieldConstants.OdometryToFieldOffsetX,6.12-FieldConstants.OdometryTOFieldOffsety, new Rotation2d(Math.PI))};
        public static Pose2d SemiAutoGoPoseBlue[]={new Pose2d(15.69-FieldConstants.OdometryToFieldOffsetX,6.12-FieldConstants.OdometryTOFieldOffsety, new Rotation2d(0))};
        
        
        public static Pose2d SemiAutoPutPoseRed[]={new Pose2d(14.72-8.23,5.17-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,4.36-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,3.88-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,3.29-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,2.73-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,2.20-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,1.61-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,1.02-4.01,new Rotation2d(Math.PI)),
                                                    new Pose2d(14.72-8.23,0.46-4.01,new Rotation2d(Math.PI))};

        public static Pose2d SemiAutoPutPoseBlue[]={new Pose2d(1.72-8.23,5.17-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,4.36-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,3.88-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,3.29-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,2.73-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,2.20-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,1.61-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,1.02-4.01,new Rotation2d(0)),
                                                    new Pose2d(1.72-8.23,0.46-4.01,new Rotation2d(0)),};
        
                                                }
    public static final class AutoConstants{
        
        public static final Command AutoRedCommand[]={
            
              new SequentialCommandGroup(new SemiAutoSwerveControll(new Pose2d(14.72-FieldConstants.OdometryToFieldOffsetX, 4.36-FieldConstants.OdometryTOFieldOffsety, new Rotation2d(Math.PI)), 1).raceWith(new WaitCommand(2)),
                                        new InstantCommand(()-> 
                                        RobotContainer.m_Arm.SetState(125, true)),
                                        new WaitCommand(1),
                                        new InstantCommand(()->RobotContainer.m_Claw.Ease()),
                                        new WaitCommand(1),
                                        new InstantCommand(()->RobotContainer.m_Arm.SetState(12, false)))
                                        ,
                new SequentialCommandGroup(new SemiAutoSwerveControll(new Pose2d(14.72-FieldConstants.OdometryToFieldOffsetX, 4.36-FieldConstants.OdometryTOFieldOffsety, new Rotation2d(Math.PI)), 1).raceWith(new WaitCommand(2)),
                                           new InstantCommand(()-> 
                                           RobotContainer.m_Arm.SetState(125, true)),
                                           new WaitCommand(1),
                                           new InstantCommand(()->RobotContainer.m_Claw.Ease()),
                                           new WaitCommand(0.5),
                                           new InstantCommand(()->RobotContainer.m_Arm.SetState(12, false)),
                                           new SemiAutoSwerveControll(new Pose2d(10-FieldConstants.OdometryToFieldOffsetX,0.924-FieldConstants.OdometryTOFieldOffsety,new Rotation2d(Math.PI)),1).raceWith(new WaitCommand(3)),
                                           new InstantCommand(()->RobotContainer.m_Arm.SetState(-40, true)),
                                           new WaitCommand(1),
                                           new InstantCommand(()->RobotContainer.m_Claw.Close()),
                                           new WaitCommand(0.5),
                                           new SemiAutoSwerveControll(new Pose2d(14.72-FieldConstants.OdometryToFieldOffsetX,0.99-FieldConstants.OdometryTOFieldOffsety,new Rotation2d(Math.PI)),1).raceWith(new WaitCommand(3)),
                                           new InstantCommand(()-> 
                                            RobotContainer.m_Arm.SetState(125, true)),
                                            new WaitCommand(1),
                                            new InstantCommand(()->RobotContainer.m_Claw.Ease()),
                                            new WaitCommand(1),
                                            new InstantCommand(()->RobotContainer.m_Arm.SetState(12, false)),
                                            new WaitCommand(1),
                                            new SemiAutoSwerveControll(new Pose2d(10-FieldConstants.OdometryToFieldOffsetX,0.924-FieldConstants.OdometryTOFieldOffsety,new Rotation2d(Math.PI)),1).raceWith(new WaitCommand(3))
                                           
                                            
                                        
                                           
                                           )
           
             
            
        };
        public static final Command AutoBlueCommand[]=
        {
            new SequentialCommandGroup(new SemiAutoSwerveControll(new Pose2d(1.73-FieldConstants.OdometryToFieldOffsetX, 4.98-FieldConstants.OdometryTOFieldOffsety, new Rotation2d(0)), 1),
            new InstantCommand(()-> 
            RobotContainer.m_Arm.SetState(140, true)),
            new WaitCommand(1),
            new InstantCommand(()->RobotContainer.m_Claw.Ease()),
            new WaitCommand(0.3),
            new InstantCommand(()->RobotContainer.m_Arm.SetState(12, false))), 
            
            new SequentialCommandGroup(new SemiAutoSwerveControll(new Pose2d(1.73-FieldConstants.OdometryToFieldOffsetX, 4.36-FieldConstants.OdometryTOFieldOffsety, new Rotation2d(Math.PI)), 1).raceWith(new WaitCommand(2)),
            new InstantCommand(()-> 
            RobotContainer.m_Arm.SetState(125, true)),
            new WaitCommand(1),
            new InstantCommand(()->RobotContainer.m_Claw.Ease()),
            new WaitCommand(0.5),
            new InstantCommand(()->RobotContainer.m_Arm.SetState(12, false)),
            new SemiAutoSwerveControll(new Pose2d(6.43-FieldConstants.OdometryToFieldOffsetX,0.924-FieldConstants.OdometryTOFieldOffsety,new Rotation2d(Math.PI)),1).raceWith(new WaitCommand(3)),
            new InstantCommand(()->RobotContainer.m_Arm.SetState(-40, true)),
            new WaitCommand(1),
            new InstantCommand(()->RobotContainer.m_Claw.Close()),
            new WaitCommand(0.5),
            new SemiAutoSwerveControll(new Pose2d(14.72-FieldConstants.OdometryToFieldOffsetX,0.99-FieldConstants.OdometryTOFieldOffsety,new Rotation2d(Math.PI)),1).raceWith(new WaitCommand(3)),
            new InstantCommand(()-> 
             RobotContainer.m_Arm.SetState(125, true)),
             new WaitCommand(1),
             new InstantCommand(()->RobotContainer.m_Claw.Ease()),
             new WaitCommand(1),
             new InstantCommand(()->RobotContainer.m_Arm.SetState(12, false)),
             new WaitCommand(1),
             new SemiAutoSwerveControll(new Pose2d(10-FieldConstants.OdometryToFieldOffsetX,0.924-FieldConstants.OdometryTOFieldOffsety,new Rotation2d(Math.PI)),1).raceWith(new WaitCommand(3)))

            

        };
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
    
        public static double kMaxSpeed = 8;//The unit is meters per second 轮子的最大转速，单位是m/s
    
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(//创建一个坐标系，上面的每一个点就是底盘上的一个drivemodule
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, -kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, -kWidth / 2.0));
    
        public static final double wheelCircumference = kWheelDiameter * Math.PI;//轮子周长

        public static final SwerveModulePosition[] m_InitialSwerveModulePositions={new SwerveModulePosition()};

        public static final double AprilTagLockTime =0.05;//apriltag在视野里出现多少秒就重新校准odometry
    }
    public static final class ControllerConstants{
        public static JoystickButton CarLockButton=new JoystickButton(RobotContainer.m_driverController, 3);
        public static JoystickButton AutoClimbChargeStationButton=new JoystickButton(RobotContainer.m_driverController,4);
        public static JoystickButton SemiAutoBackButton=new JoystickButton(RobotContainer.m_driverController, 10);
        public static JoystickButton SemiAutoGoButton=new JoystickButton(RobotContainer.m_driverController, 9);
    }
}
