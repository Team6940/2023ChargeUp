package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SemiAutoConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * 这个指令会接管driver的控制，然后把机器挪到指定的位置
 */
public class SemiAutoSwerveControll extends CommandBase{
    TrapezoidProfile m_SemiAutoVelocityProfile;//用来描述自动移动速度的梯形曲线
    TrapezoidProfile m_SemiAutoOmegaProfile;//用来描述自动移动的角速度的梯形曲线曲线
    Rotation2d m_MoveAngle;//我车子的移动方向
    double m_BeginTime;
    public Pose2d m_TargetPose2d;//我想要移动到的姿态
    /**
     * 这个指令会接管driver的控制，然后把机器挪到指定的姿态
     * @param _TargetPose2d 目标姿态
     */
    public SemiAutoSwerveControll(Pose2d _TargetPose2d)
    {

        addRequirements(RobotContainer.m_SwerveBase);
        Pose2d _NowPose2d=RobotContainer.m_SwerveBase.getPose();
        m_TargetPose2d=_TargetPose2d;
        m_MoveAngle=_TargetPose2d.minus(_NowPose2d).getRotation();
        double _Distance=Math.sqrt((_NowPose2d.getX()-_TargetPose2d.getX())*(_NowPose2d.getX()-_TargetPose2d.getX())+
        (_NowPose2d.getY()-_TargetPose2d.getY())*(_NowPose2d.getY()-_TargetPose2d.getY()));//计算两个点之间的距离
        SmartDashboard.putNumber("Distance", _Distance);
         m_SemiAutoVelocityProfile=new TrapezoidProfile(SemiAutoConstants.kSemiAutoVelocityConstrants, 
                                            new TrapezoidProfile.State(_Distance,0),
                                            new TrapezoidProfile.State(0, 0.0));//创建梯形移动曲线
         m_SemiAutoOmegaProfile=new TrapezoidProfile(SemiAutoConstants.kSemiAutoOmegaConstrants, 
                                               new TrapezoidProfile.State(_TargetPose2d.getRotation().getRadians(),0),
                                            new TrapezoidProfile.State(_NowPose2d.getRotation().getRadians(), 0.0));

    }
    @Override
    public void initialize() {
        m_BeginTime=Timer.getFPGATimestamp();
        SmartDashboard.putBoolean("IsSemiAuto",true);
    }
    @Override
    public void execute() {
        double _DeltaTime=Timer.getFPGATimestamp()-m_BeginTime;
        double _Velocity=m_SemiAutoVelocityProfile.calculate(_DeltaTime).velocity;
        double _Omega=m_SemiAutoOmegaProfile.calculate(_DeltaTime).velocity;
        RobotContainer.m_SwerveBase.Drive(new Translation2d(_Velocity,m_MoveAngle), _Omega, true, false);
        SmartDashboard.putBoolean("IsSemiAuto",true);
        SmartDashboard.putNumber("Omega", _Omega);
        SmartDashboard.putNumber("Velocity",_Velocity);
        SmartDashboard.putNumber("DeltaTIme", _DeltaTime);
    }
    @Override
    public void end(boolean _Interuppted)
    {
        SmartDashboard.putBoolean("IsSemiAuto",false);
    }
    @Override
    public boolean isFinished()
    {
        return !RobotContainer.m_driverController.getAButton();
    }
}
