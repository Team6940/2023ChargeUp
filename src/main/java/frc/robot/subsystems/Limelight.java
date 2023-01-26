package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.library.team1706.LinearInterpolationTable;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    
  private static Limelight instance = null;
    NetworkTable m_LimelightTable;
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
      _BotPose=m_LimelightTable.getEntry("botpose").getDoubleArray(_BotPose);
      return new Pose2d(_BotPose[0],_BotPose[1],Rotation2d.fromDegrees(_BotPose[6]));
    }
    /**
     * 获取机器人视野里是否识别到目标
     * @return  视野里是否有目标，有为true 没有为false
     */
    public boolean IsTargetLocked()
    {
      boolean _IsTargetLocked;
      _IsTargetLocked=m_LimelightTable.getEntry("tv").getBoolean(false);
      return _IsTargetLocked;
    }
}
