package frc.robot.commands;


import javax.lang.model.util.ElementScanner14;

import org.w3c.dom.ranges.RangeException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SemiAutoConstants;
import frc.robot.library.NumberLimiter.NumberLimiter;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GameConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * 这个指令会接管driver的控制，然后把机器挪到指定的位置
 */
public class SemiAutoSwerveControll extends CommandBase{
    PIDController m_SemiAutoVelocityPIDController;//用来描述自动移动速度的梯形曲线
    PIDController m_SemiAutoOmegaPIDcontroller;//用来描述自动移动的角速度的梯形曲线曲线
    Rotation2d m_MoveAngle;//我车子的移动方向
    double m_BeginTime;
    Trigger FinishTrigger;
    SlewRateLimiter m_SemiAutoOmegSlewRateLimiter=new SlewRateLimiter(DriveConstants.kRotationSlew);
    static boolean m_IsSemiAuto;
    public Pose2d m_TargetPose2d;//我想要移动到的姿态
    /**
     * 这个指令会接管driver的控制，然后把机器挪到指定的姿态
     * @param _TargetPose2d 目标姿态
     */
    public SemiAutoSwerveControll(Pose2d _TargetPose2d,Trigger _FinishTrigger)
    {
        
        m_TargetPose2d=_TargetPose2d;
        addRequirements(RobotContainer.m_SwerveBase);

    }
    @Override
    public void initialize() {
        m_BeginTime=Timer.getFPGATimestamp();
        m_IsSemiAuto=true;
        SmartDashboard.putBoolean("IsSemiAuto",true);
        Pose2d _NowPose2d=RobotContainer.m_SwerveBase.getPose();
        m_MoveAngle=new Rotation2d(m_TargetPose2d.minus(_NowPose2d).getX(),m_TargetPose2d.minus(_NowPose2d).getY());
        double _Distance=Math.sqrt((_NowPose2d.getX()-m_TargetPose2d.getX())*(_NowPose2d.getX()-m_TargetPose2d.getX())+
        (_NowPose2d.getY()-m_TargetPose2d.getY())*(_NowPose2d.getY()-m_TargetPose2d.getY()));//计算两个点之间的距离
        // SmartDashboard.putNumber("Distance", _Distance);
        // SmartDashboard.putBoolean("IsSemiAuto",false);
         m_SemiAutoVelocityPIDController=new PIDController(SemiAutoConstants.kSemiAutoVelocityP, SemiAutoConstants.kSemiAutoVelocityI, SemiAutoConstants.kSemiAutoVelocityD);//创建梯形移动曲线
        m_SemiAutoOmegaPIDcontroller=new PIDController(SemiAutoConstants.kSemiAutoOmegaP,SemiAutoConstants.kSemiAutoOmegaI, SemiAutoConstants.kSemiAutoOmegaD);
        m_SemiAutoOmegaPIDcontroller.setSetpoint(0);
        m_SemiAutoOmegaPIDcontroller.setTolerance(0.05);
        m_SemiAutoVelocityPIDController.setSetpoint(0);
        m_SemiAutoVelocityPIDController.setTolerance(0.05);
        // m_SemiAutoOmegaProfile.setGoal(m_TargetPose2d.minus(_NowPose2d).getRotation().getRadians());
        // m_SemiAutoOmegaProfile.reset(_NowPose2d.getRotation().getRadians());
    }
    @Override
    public void execute() {
        Pose2d _NowPose2d=RobotContainer.m_SwerveBase.getPose();
        m_MoveAngle=new Rotation2d(m_TargetPose2d.minus(_NowPose2d).getX(),m_TargetPose2d.minus(_NowPose2d).getY());
        double _DeltaTime=Timer.getFPGATimestamp()-m_BeginTime;
        
        double _Distance=Math.sqrt((_NowPose2d.getX()-m_TargetPose2d.getX())*(_NowPose2d.getX()-m_TargetPose2d.getX())+
        (_NowPose2d.getY()-m_TargetPose2d.getY())*(_NowPose2d.getY()-m_TargetPose2d.getY()));//计算两个点之间的距离
        double _Velocity=m_SemiAutoVelocityPIDController.calculate(_Distance);

        double _Omega=m_SemiAutoOmegSlewRateLimiter.calculate(m_SemiAutoOmegaPIDcontroller.calculate(-m_TargetPose2d.minus(_NowPose2d).getRotation().getRadians()));
         _Omega=NumberLimiter.Limit(SemiAutoConstants.SemiAutoOmegaMax,-SemiAutoConstants.SemiAutoOmegaMax,_Omega);
        _Velocity=NumberLimiter.Limit(SemiAutoConstants.kSemiAutoVelocityConstrants.maxVelocity,-SemiAutoConstants.kSemiAutoVelocityConstrants.maxVelocity,_Velocity);
         RobotContainer.m_SwerveBase.Drive(new Translation2d(_Velocity,m_MoveAngle), _Omega, true, false);
        SmartDashboard.putBoolean("IsSemiAuto",true);
        SmartDashboard.putNumber("MoveAngle", m_MoveAngle.getDegrees());
        SmartDashboard.putNumber("Velocity",_Velocity);
        SmartDashboard.putNumber("DeltaTIme", _DeltaTime);
        SmartDashboard.putNumber("Omega", _Omega);
    }
    public static boolean IsSemiAuto()
    {
        return m_IsSemiAuto;
    }
    @Override
    public void end(boolean _Interuppted)
    {
        SmartDashboard.putBoolean("IsSemiAuto",false);
        m_IsSemiAuto=false;
    }
    @Override
    public boolean isFinished()
    {
        return !RobotContainer.m_driverController.getRightStickButton()||(m_SemiAutoOmegaPIDcontroller.atSetpoint()&&m_SemiAutoVelocityPIDController.atSetpoint());
    }
    public static Command GenerateSemiAutoCommand(Pose2d _NowPose,Boolean _IsBack,int _SelectedPutPlace)
    {
        Pose2d _TargetPose=new Pose2d();
        
        for(int i=0;i<GameConstants.Areas.length;++i)
        {
            if(_NowPose.getX()-FieldConstants.OdometryToFieldOffsetX>=GameConstants.Areas[i][0]
            &&_NowPose.getX()-FieldConstants.OdometryToFieldOffsetX<GameConstants.Areas[i][3]
            &&_NowPose.getY()-FieldConstants.OdometryTOFieldOffsety<GameConstants.Areas[i][1]
            &&_NowPose.getY()-FieldConstants.OdometryTOFieldOffsety>=GameConstants.Areas[i][2])
            {
                if(DriverStation.getAlliance()==Alliance.Red)
                {
                    if(_IsBack)
                    {
                        _TargetPose=GameConstants.SemiAutoBackPoseRed[i];
                    }
                    else
                    {
                        _TargetPose=GameConstants.SemiAutoGoPoseRed[i];
                    }
                }
                else
                {
                    if(_IsBack)
                    {
                        _TargetPose=GameConstants.SemiAutoBackPoseBlue[i];
                    }
                    else
                    {
                        _TargetPose=GameConstants.SemiAutoGoPoseBlue[i];
                    }
                }
                }
        }
        if(_TargetPose==new Pose2d())
        {
            if(_IsBack)
                if(DriverStation.getAlliance()==Alliance.Red)
                    return new SemiAutoSwerveControll(GameConstants.SemiAutoPutPoseRed[_SelectedPutPlace],ControllerConstants.SemiAutoBackButton);
                else
                    return new SemiAutoSwerveControll(GameConstants.SemiAutoPutPoseBlue[_SelectedPutPlace],ControllerConstants.SemiAutoBackButton);
            else
                if(DriverStation.getAlliance()==Alliance.Red)
                    return new SemiAutoSwerveControll(GameConstants.SemiAutoBackPoseRed[1],ControllerConstants.SemiAutoGoButton);
                else 
                    return new SemiAutoSwerveControll(GameConstants.SemiAutoBackPoseBlue[1],ControllerConstants.SemiAutoGoButton);
        }
        if(_IsBack)
             return new SemiAutoSwerveControll(_TargetPose,ControllerConstants.SemiAutoBackButton);
        else
            return new SemiAutoSwerveControll(_TargetPose, ControllerConstants.SemiAutoGoButton);
    }
}
