package frc.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain 
{
    private final int RT_BACK_DRIVE_ID = 5;
    private final int RT_FRNT_DRIVE_ID = 7;
    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;

    private final SwerveModule RT_FRNT_STEER;
    //private final SteeringCAN RT_BACK_STEER;
    //private final Steering LT_FRNT_STEER;
    //private final Steering LT_BACK_STEER;

    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_BACK_STEER_ID = 6;
    private final int RT_FRNT_STEER_ID = 8;

    private final int MAG_ENC_PORT = 9;
    private final int CAN_ENC_CAN_ID = 9;

    private double joystickAngle;

    public Drivetrain()
    {
        RT_FRNT_STEER = new SwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, MAG_ENC_PORT);

        RT_FRNT_STEER.resetMagEnc();
    }

    public void drive(double xJoy, double yJoy)
    {
        joystickAngle = Math.atan(Math.abs(xJoy) / Math.abs(yJoy));
        joystickAngle *= (180 / Math.PI);

        if(yJoy <= 0)   //joystick pointed up
        {
            if(xJoy < 0)    //joystick pointed left
            {
                //no change
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                joystickAngle = 360 - joystickAngle; 
            }
        }
        else    //joystick pointed down
        {
            if(xJoy < 0)    //joystick pointed left
            {
                joystickAngle = 180 - joystickAngle;
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                joystickAngle = 180 + joystickAngle;
            }
        }

        RT_FRNT_STEER.setWheelRotation(joystickAngle);

        setDrivePower(Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
    }

    public void setSteerPower(double pwr)
    {
        RT_FRNT_STEER.setSteerPower(pwr);
    }

    public void setDrivePower(double pwr)
    {
        RT_FRNT_STEER.setDrivePower(pwr);
    }

    public void setBrakeMode()
    {
        RT_FRNT_STEER.setBrakeMode();
    }
    public void setCoastMode()
    {
        RT_FRNT_STEER.setCoastMode();
    }

    public void updateShuffleboard()
    {
        SmartDashboard.putNumber("Joystick", joystickAngle);

        RT_FRNT_STEER.updateShuffleboard();
    }
}
