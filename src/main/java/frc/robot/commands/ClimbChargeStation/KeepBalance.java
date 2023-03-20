package frc.robot.commands.ClimbChargeStation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbConstants;

public class KeepBalance extends CommandBase{

    PIDController m_BalancePIDController;
    public KeepBalance()
    {
        addRequirements(RobotContainer.m_SwerveBase);        
    }
    /**
     * 将陀螺仪上的以x y轴上旋转表现的形式转化为以θ，α表现的形式，其中θ表示旋转的方向，alpha表示旋转的程度，单位均为角度
     * @param _X x轴上的旋转，旋转路径垂直于x轴
     * @param _Y y轴上的旋转，旋转路径垂直于y轴
     * @return 一个二元组（θ，α）
     */
    double[] Transform(double _X,double _Y)
    {
        double[] _Transformed={0,_Y};
        return _Transformed;
    }
    @Override
    public void initialize()
    {
        m_BalancePIDController=new PIDController(ClimbConstants.ClimbPIDControllerkP, ClimbConstants.ClimbPIDControllerkI, ClimbConstants.ClimbPIDControllerkD);
        m_BalancePIDController.setSetpoint(0);
    }
    @Override
    public void execute()
    {
        double[] _PigeonGyroData={0,0,0};
        RobotContainer.m_SwerveBase.m_Gyro.getAccumGyro(_PigeonGyroData);
        double _LeanX=_PigeonGyroData[0];
        double _LeanY=_PigeonGyroData[1];
        double[] _TransformedData=Transform(_LeanX,_LeanY);
        Translation2d _DriveVector=new Translation2d(m_BalancePIDController.calculate(_TransformedData[0]),new Rotation2d(_TransformedData[0]));
        RobotContainer.m_SwerveBase.Drive(_DriveVector, 0, false, false);
    }
    @Override
    public boolean isFinished()
    {
        return !RobotContainer.m_driverController.getYButton();
    }
}
