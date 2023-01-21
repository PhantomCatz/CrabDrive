package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzSwerveModule
{
    private final WPI_TalonFX STEERING_MOTOR;
    private final WPI_TalonFX DRIVE_MOTOR;

    private final int motorID;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncDigitalInput;

    private PIDController pid;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double currentAngle = 0.0;
    private double angleError = 0.0;
    private double flippedAngleError = 0.0;

    private double command;
    private boolean driveDirectionFlipped = false;

    private final double WHEEL_OFFSET;

    //current limiting
    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderID, double offset)
    {
        STEERING_MOTOR = new WPI_TalonFX(steerMotorID);
        DRIVE_MOTOR = new WPI_TalonFX(driveMotorID);

        //Set current limit
        swerveModuleCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        STEERING_MOTOR.configSupplyCurrentLimit(swerveModuleCurrentLimit);
        DRIVE_MOTOR.configSupplyCurrentLimit(swerveModuleCurrentLimit);

        STEERING_MOTOR.configFactoryDefault();
        DRIVE_MOTOR.configFactoryDefault();
        DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        
        MagEncDigitalInput = new DigitalInput(encoderID);
        magEnc = new DutyCycleEncoder(MagEncDigitalInput);

        pid = new PIDController(kP, kI, kD);

        WHEEL_OFFSET = offset;

        motorID = steerMotorID;

        setCoastMode();
    }
    public void updateShuffleboard()
    {
        SmartDashboard.putNumber(motorID + " Mag Encoder", magEnc.get() );//* 360.0);
        SmartDashboard.putNumber(motorID + " Wheel Angle", ((magEnc.get() - WHEEL_OFFSET) * 360.0));
        //SmartDashboard.putBoolean(motorID + " Flipped", driveDirectionFlipped);
    }

    public void resetMagEnc()
    {
        magEnc.reset();
    }

    public void setBrakeMode()
    {
        STEERING_MOTOR.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoastMode()
    {
        STEERING_MOTOR.setNeutralMode(NeutralMode.Coast);
    }

    public double closestAngle(double a, double b)
    {
        // get direction
        double dir = b % 360.0 - a % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
                //closest angle shouldn't be more than 180 degrees. If it is, use other direction
                if(dir > 180.0)
                {
                    dir -= 360;
                }
        }

        return dir;
    }

    public void setWheelRotation(double target)
    {
        currentAngle = (magEnc.get() - WHEEL_OFFSET) * 360.0;
        // find closest angle to target angle
        angleError = closestAngle(currentAngle, target);
        //SmartDashboard.putNumber(motorID + " Angle Error", angleError);
        // find closest angle to target angle + 180
        flippedAngleError = closestAngle(currentAngle, target + 180.0);
        //SmartDashboard.putNumber(motorID + " Flip Angle Error", flippedAngleError);
        // if the closest angle to target is shorter
        if (Math.abs(angleError) <= Math.abs(flippedAngleError))
        {
            driveDirectionFlipped = false;
            command = pid.calculate(currentAngle, currentAngle + angleError);
        }
        // if the closest angle to target + 180 is shorter
        else
        {
            driveDirectionFlipped = true;
            command = pid.calculate(currentAngle, currentAngle + flippedAngleError);
        }

        command = -command / (180 * kP); //scale down command to a range of -1 to 1
        STEERING_MOTOR.set(ControlMode.PercentOutput, command);
    }

    public void setSteerPower(double pwr)
    {
        STEERING_MOTOR.set(ControlMode.PercentOutput, pwr);
    }

    public void setDrivePower(double pwr)
    {
        pwr *= 0.5;

        if(driveDirectionFlipped == true)
        {
            pwr = -pwr;
        }
        DRIVE_MOTOR.set(ControlMode.PercentOutput, -pwr);
    }
    
    public double getAngle()
    {
        return currentAngle;
    }
    public double getError()
    {
        return angleError;
    }
    public double getFlipError()
    {
        return flippedAngleError;
    }
}
