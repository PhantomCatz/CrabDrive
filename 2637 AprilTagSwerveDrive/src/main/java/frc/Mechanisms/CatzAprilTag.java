package frc.Mechanisms;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzAprilTag
{
    public double[] defultArray = {999.0,999.0,999.0,999.0,999.0,999.0};

    public double[] robotPosition = defultArray;

    private final double METERTOINCH = 39.37;
    private final double TAGTOTAG = 570.32;

    public CatzLog data;

    
    public CatzAprilTag()
    {

    }

    public void positionUpdate()
    {
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(robotPosition).length == 6)
        {
            for(int i = 0; i < 6; i++)
            {
                if(i<3){
                    robotPosition[i] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(robotPosition)[i]*METERTOINCH;
                }
                    
                else{
                    robotPosition[i] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(robotPosition)[i];
                }
            }
        }else
        {
            robotPosition = defultArray;
        }
    }

    public double[] getBotPose()
    {
        return robotPosition;
    }

    public double getDisToWall()
    {
        if(robotPosition[0] != 999.0)
            return TAGTOTAG/2-Math.abs(robotPosition[0]);
        else
            return -1.0;
    }

    public void smartDashboardAprilTag()
    {
        SmartDashboard.putNumber("botpos Px", robotPosition[0]);
        SmartDashboard.putNumber("botpos Py", robotPosition[1]);
        SmartDashboard.putNumber("botpos Pz", robotPosition[2]);
        SmartDashboard.putNumber("botpos Rx", robotPosition[3]);
        SmartDashboard.putNumber("botpos Ry", robotPosition[4]);
        SmartDashboard.putNumber("botpos Rz", robotPosition[5]);
        SmartDashboard.putNumber("Dx to tagwall", TAGTOTAG/2-Math.abs(robotPosition[0]));
    }

    // public void dataCollection()
    // {
    //     if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_APRILTAG)
    //     {
    //         data = new CatzLog(Robot.currentTime.get(), robotPosition[0], robotPosition[1], robotPosition[2], robotPosition[3], robotPosition[4], robotPosition[5]);  
    //         Robot.dataCollection.logData.add(data);
    //         System.out.println("data collected");
    //     }
    // }
}