package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule
{
    private final WPI_TalonFX STEERING_MOTOR;
    private final WPI_TalonFX DRIVE_MOTOR;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncDigitalInput;

    private PIDController pid;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double command;

    private double angleError = 0.0;
    private double currentEncPos = 0.0;

    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID)
    {
        STEERING_MOTOR = new WPI_TalonFX(steerMotorID);
        DRIVE_MOTOR = new WPI_TalonFX(driveMotorID);

        STEERING_MOTOR.configFactoryDefault();
        DRIVE_MOTOR.configFactoryDefault();
        DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        
        MagEncDigitalInput = new DigitalInput(encoderID);
        magEnc = new DutyCycleEncoder(MagEncDigitalInput);

        pid = new PIDController(kP, kI, kD);

        setCoastMode();
    }
    public void updateShuffleboard()
    {
        SmartDashboard.putNumber("Mag Encoder", magEnc.get());
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

    public void setWheelRotation(double targetAngle)
    {
        currentEncPos  = magEnc.get();

        currentEncPos = (currentEncPos % 1.0);

        if(currentEncPos < 0.0)
        {
            currentEncPos = (currentEncPos + 1) * 360.0;
        }
        else
        {
            currentEncPos *= 360.0;
        }

        angleError = Math.abs(targetAngle - currentEncPos);

        if(angleError > 180)
        {
            if(currentEncPos < 180)
            {
                currentEncPos += 360.0;
            }
            else if(currentEncPos > 180)
            {
                currentEncPos -= 360.0;
            }
        }

        command = pid.calculate(currentEncPos, targetAngle);
        command = -command / (180 * kP);

        STEERING_MOTOR.set(ControlMode.PercentOutput, command);
    }

    public void setSteerPower(double pwr)
    {
        STEERING_MOTOR.set(ControlMode.PercentOutput, pwr);
    }

    public void setDrivePower(double pwr)
    {
        pwr *= -0.5;
        DRIVE_MOTOR.set(ControlMode.PercentOutput, pwr);
    }
}
