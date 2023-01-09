package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Driving 
{
    private final WPI_TalonFX LT_FRNT_DRIVE;
    private final WPI_TalonFX RT_FRNT_DRIVE;
    private final WPI_TalonFX LT_BACK_DRIVE;
    private final WPI_TalonFX RT_BACK_DRIVE;

    private final int LT_FRNT_ID = 1;
    private final int LT_BACK_ID = 3;
    private final int RT_BACK_ID = 5;
    private final int RT_FRNT_ID = 7;

    public Driving()
    {
        LT_FRNT_DRIVE = new WPI_TalonFX(LT_FRNT_ID);
        LT_BACK_DRIVE = new WPI_TalonFX(LT_BACK_ID);
        RT_BACK_DRIVE = new WPI_TalonFX(RT_BACK_ID);
        RT_FRNT_DRIVE = new WPI_TalonFX(RT_FRNT_ID);

        LT_FRNT_DRIVE.configFactoryDefault();
        LT_BACK_DRIVE.configFactoryDefault();
        RT_BACK_DRIVE.configFactoryDefault();
        RT_FRNT_DRIVE.configFactoryDefault();

        LT_FRNT_DRIVE.setNeutralMode(NeutralMode.Brake);
        LT_BACK_DRIVE.setNeutralMode(NeutralMode.Brake);
        RT_BACK_DRIVE.setNeutralMode(NeutralMode.Brake);
        RT_FRNT_DRIVE.setNeutralMode(NeutralMode.Brake);
    }
    public void setMotorPower(double pwr)
    {
        LT_FRNT_DRIVE.set(ControlMode.PercentOutput, pwr);
        LT_BACK_DRIVE.set(ControlMode.PercentOutput, pwr);
        RT_FRNT_DRIVE.set(ControlMode.PercentOutput, pwr);
        RT_BACK_DRIVE.set(ControlMode.PercentOutput, pwr);
    }
}
