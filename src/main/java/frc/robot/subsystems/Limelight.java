package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    
  private static Limelight instance = null;
    public NetworkTable m_LimelightTable;
    public Limelight() 
    {
      m_LimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }
    
     public static Limelight getInstance() 
    {
      if (instance == null)
      {
          instance = new Limelight();  
      }
        return instance;
      }
    /**
     * 获取机器人用limelight反解出来的位置
     * @return
     */
    public Pose2d GetPose2dBotPose()
    {
      double[] _BotPose={0,0,0,0,0,0};
      if(m_LimelightTable.getEntry("botpose").getDoubleArray(_BotPose).length==6)
      _BotPose=m_LimelightTable.getEntry("botpose").getDoubleArray(_BotPose);
      return new Pose2d(_BotPose[0],_BotPose[1],Rotation2d.fromDegrees(_BotPose[5]));
    }
    /**
     * 获取机器人视野里是否识别到目标
     * @return  视野里是否有目标，有为true 没有为false
     */
    public double IsTargetLocked()
    {
      double _IsTargetLocked;
      _IsTargetLocked=m_LimelightTable.getEntry("tv").getDouble(0.0);
      return _IsTargetLocked;
    }
    @Override
    public void periodic()
    {
      // SmartDashboard.putNumber("IsTargetLocked",IsTargetLocked());
      if(IsTargetLocked()==1.0)
       SmartDashboard.putNumber("BotPoseX", GetPose2dBotPose().getX());
    }
}
