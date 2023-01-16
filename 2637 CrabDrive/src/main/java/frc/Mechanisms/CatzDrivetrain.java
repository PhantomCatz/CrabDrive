package frc.Mechanisms;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CatzDrivetrain 
{
    private final CatzSwerveModule RT_FRNT_MODULE;
    private final CatzSwerveModule RT_BACK_MODULE;
    private final CatzSwerveModule LT_FRNT_MODULE;
    private final CatzSwerveModule LT_BACK_MODULE;

    private AHRS navX;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_BACK_DRIVE_ID = 5;
    private final int RT_FRNT_DRIVE_ID = 7;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_BACK_STEER_ID = 6;
    private final int RT_FRNT_STEER_ID = 8;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_FRNT_ENC_PORT = 8;
    private final int RT_BACK_ENC_PORT = 7;

    private final double LT_FRNT_OFFSET = 0.012;
    private final double LT_BACK_OFFSET = -0.627;
    private final double RT_FRNT_OFFSET = 0.668;
    private final double RT_BACK_OFFSET = 0.072;

    private double joystickAngle;

    public CatzDrivetrain()
    {
        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET);

        navX = new AHRS();
        navX.reset();

        LT_FRNT_MODULE.resetMagEnc();
        LT_BACK_MODULE.resetMagEnc();
        RT_FRNT_MODULE.resetMagEnc();
        RT_BACK_MODULE.resetMagEnc();
    }

    public void testAngle()
    {
        System.out.println("Positive: " + LT_FRNT_MODULE.closestAngle(359.0, 361.0));

        System.out.println("Negative: " + LT_FRNT_MODULE.closestAngle(-359.0, -361.0));
    }

    public void drive(double xJoy, double yJoy)
    {
        joystickAngle = calculateJoystickAngle(xJoy, yJoy);

        //LT_FRNT_MODULE.setWheelRotation(joystickAngle);
        //LT_BACK_MODULE.setWheelRotation(joystickAngle);
        //RT_FRNT_MODULE.setWheelRotation(joystickAngle);
        RT_BACK_MODULE.setWheelRotation(joystickAngle);

        setDrivePower(Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
    }

    public void rotateInPlace(double pwr)
    {
        LT_FRNT_MODULE.setWheelRotation(135.0);
        LT_BACK_MODULE.setWheelRotation(45.0);
        RT_FRNT_MODULE.setWheelRotation(315.0);
        RT_BACK_MODULE.setWheelRotation(225.0);

        pwr *= 0.6;

        LT_FRNT_MODULE.setDrivePower(pwr);
        LT_BACK_MODULE.setDrivePower(pwr);
        RT_FRNT_MODULE.setDrivePower(pwr);
        RT_BACK_MODULE.setDrivePower(pwr);
    }

    public void translateTurn(double direction, double translatePower, double turnPower)
    {
    double turnAngle = turnPower * 45.0;

    // if the left front wheel is in the front
    if (LT_FRNT_MODULE.closestAngle(direction, 135.0) >= 90.0)
    {
        LT_FRNT_MODULE.setWheelRotation(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        LT_FRNT_MODULE.setWheelRotation(direction - turnAngle);
    }
    // if the left back wheel is in the front
    if (LT_BACK_MODULE.closestAngle(direction, 225.0) > 90.0)
    {
        LT_BACK_MODULE.setWheelRotation(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        LT_BACK_MODULE.setWheelRotation(direction - turnAngle);
    }
    // if the right front wheel is in the front
    if (RT_FRNT_MODULE.closestAngle(direction, 45.0) > 90.0)
    {
        RT_FRNT_MODULE.setWheelRotation(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        RT_FRNT_MODULE.setWheelRotation(direction - turnAngle);
    }
    // if the right back wheel is in the front
    if (RT_BACK_MODULE.closestAngle(direction, 315.0) >= 90.0)
    {
        RT_BACK_MODULE.setWheelRotation(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        RT_BACK_MODULE.setWheelRotation(direction - turnAngle);
    }

        LT_FRNT_MODULE.setDrivePower(translatePower);
        LT_BACK_MODULE.setDrivePower(translatePower);
        RT_FRNT_MODULE.setDrivePower(translatePower);
        RT_BACK_MODULE.setDrivePower(translatePower);
    }
    

    public double calculateJoystickAngle(double xJoy, double yJoy)
    {
        double angle = Math.atan(Math.abs(xJoy) / Math.abs(yJoy));
        angle *= (180 / Math.PI);

        if(yJoy <= 0)   //joystick pointed up
        {
            if(xJoy < 0)    //joystick pointed left
            {
                //no change
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -angle;
            }
        }
        else    //joystick pointed down
        {
            if(xJoy < 0)    //joystick pointed left
            {
                angle = 180 - angle;
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -180 + angle;
            }
        }
        return angle;
    }

    public double getGyroAngle()
    {
        return navX.getAngle();
    }

    public void setSteerPower(double pwr)
    {
        LT_FRNT_MODULE.setSteerPower(pwr);
        LT_BACK_MODULE.setSteerPower(pwr);
        RT_FRNT_MODULE.setSteerPower(pwr);
        RT_BACK_MODULE.setSteerPower(pwr);
    }

    public void setDrivePower(double pwr)
    {
        LT_FRNT_MODULE.setDrivePower(pwr);
        LT_BACK_MODULE.setDrivePower(pwr);
        RT_FRNT_MODULE.setDrivePower(pwr);
        RT_BACK_MODULE.setDrivePower(pwr);
    }

    public void setBrakeMode()
    {
        LT_FRNT_MODULE.setBrakeMode();
        LT_BACK_MODULE.setBrakeMode();
        RT_FRNT_MODULE.setBrakeMode();
        RT_BACK_MODULE.setBrakeMode();
    }
    public void setCoastMode()
    {
        LT_FRNT_MODULE.setCoastMode();
        LT_BACK_MODULE.setCoastMode();
        RT_FRNT_MODULE.setCoastMode();
        RT_BACK_MODULE.setCoastMode();
    }

    public void updateShuffleboard()
    {
        SmartDashboard.putNumber("Joystick", joystickAngle);

        LT_FRNT_MODULE.updateShuffleboard();
        LT_BACK_MODULE.updateShuffleboard();
        RT_FRNT_MODULE.updateShuffleboard();
        RT_BACK_MODULE.updateShuffleboard();
    }
}
