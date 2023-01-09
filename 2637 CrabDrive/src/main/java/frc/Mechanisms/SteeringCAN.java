package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SteeringCAN 
{
    private final WPI_TalonFX STEERING_MOTOR;

    private CANCoder canEnc;

    private PIDController pid;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double command;

    private double angleError = 0.0;
    private double currentEncPos = 0.0;

    public SteeringCAN(int driveMotorID, int steerMotorID, int encoderID)
    {
        STEERING_MOTOR = new WPI_TalonFX(steerMotorID);

        STEERING_MOTOR.configFactoryDefault();
    
        canEnc = new CANCoder(encoderID);
        canEnc.configFactoryDefault();

        pid = new PIDController(kP, kI, kD);

        setCoastMode();
    }
    public void updateShuffleboard()
    {
        SmartDashboard.putNumber("Can Encoder", canEnc.getAbsolutePosition());
        SmartDashboard.putNumber("Can Encoder Graph", canEnc.getAbsolutePosition());
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
        currentEncPos = canEnc.getAbsolutePosition();

        if(currentEncPos > 360.0)
        {
            currentEncPos -= 360.0;
        }
        else if(currentEncPos < 0.0)
        {
            currentEncPos += 360.0;
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

    public void setPower(double pwr)
    {
        STEERING_MOTOR.set(ControlMode.PercentOutput, pwr);
    }
}
