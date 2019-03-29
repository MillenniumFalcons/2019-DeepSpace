/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3647subsystems;


import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/**
 * Add your docs here.
 */
public class ShoppingCart 
{
	public static WPI_TalonSRX shoppingCartSRX = new WPI_TalonSRX(Constants.shoppingCartMotorPin);
	private static VictorSPX shoppingCartSPX = new VictorSPX(Constants.shoppingCartSPXPin); // motor for shopping cart wheels
    public static ShoppingCartPosition currentState, aimedState;
    public static int encoderError, encoderValue, encoderVelocity;
	public static double overrideValue;
	public static boolean manualOverride;

    public static void init()
	{
		//Motor & Sensor Direction
		shoppingCartSRX.setInverted(false);
		shoppingCartSRX.setSensorPhase(true);
		
		//Current Limiting
		shoppingCartSRX.configPeakCurrentLimit(Constants.kHatchWristPeakCurrent, Constants.kTimeoutMs);
		shoppingCartSRX.configPeakCurrentDuration(Constants.kHatchWristPeakCurrentDuration);
		shoppingCartSRX.configContinuousCurrentLimit(Constants.kHatchWristContinuousCurrent, Constants.kTimeoutMs);
		shoppingCartSRX.enableCurrentLimit(true);
		//PID
		shoppingCartSRX.config_kP(Constants.kHatchWristPID, Constants.kHatchWristP, Constants.kTimeoutMs);
		shoppingCartSRX.config_kI(Constants.kHatchWristPID, Constants.kHatchWristI, Constants.kTimeoutMs);
		shoppingCartSRX.config_kD(Constants.kHatchWristPID, Constants.kHatchWristD, Constants.kTimeoutMs);
		shoppingCartSRX.config_kF(Constants.kHatchWristPID, Constants.kHatchWristF, Constants.kTimeoutMs);

		//Motion Magic Constants
		shoppingCartSRX.configMotionAcceleration(Constants.kHatchWristAcceleration, Constants.kTimeoutMs);
        shoppingCartSRX.configMotionCruiseVelocity(Constants.kHatchWristCruiseVelocity, Constants.kTimeoutMs);
        resetEncoder();
    }

    public void configPIDFVA(double p, double i, double d, double f, int vel, int accel) //for configuring PID values
	{
		//PID
		shoppingCartSRX.config_kP(Constants.kHatchWristPID, p, Constants.kTimeoutMs);
		shoppingCartSRX.config_kI(Constants.kHatchWristPID, i, Constants.kTimeoutMs);
		shoppingCartSRX.config_kD(Constants.kHatchWristPID, d, Constants.kTimeoutMs);
		shoppingCartSRX.config_kF(Constants.kHatchWristPID, f, Constants.kTimeoutMs);
		
		// motion magic
		shoppingCartSRX.configMotionAcceleration(accel, Constants.kTimeoutMs);
		shoppingCartSRX.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);

	}
    
    public static enum ShoppingCartPosition
	{
		STOWED,
		DEPLOYED,
		MANUAL,
		MIDDLE
    }

    public static void run()
    {
        
        if(aimedState != null)
        {
            switch(aimedState)
            {
                case MANUAL:
                    if(!manualOverride)
                        overrideValue = 0;
                        moveManual(overrideValue);
                    break;
                case DEPLOYED:
                    deployShoppingCart();
                    break;
                case STOWED:
                    moveToStowed();
					break;
				case MIDDLE:
					setPosition(1000);
					break;
            }
        }
    }
    
	public static void runSPX(double demand)
	{
		shoppingCartSPX.set(ControlMode.PercentOutput, -demand);
	}


    public static void setManualOverride(double jValue)
	{
		if(Math.abs(jValue) > .15) //deadzone
		{
			manualOverride = true;
			aimedState = ShoppingCartPosition.MANUAL;
            overrideValue = jValue;
		} 
		else 
		{
			manualOverride = false;
		}
	}

    private static int encoderState, manualAdjustment, manualEncoderValue;
	public static void moveManual(double jValue)
	{
		updateEncoder();
		if(jValue > 0)
		{
			setOpenLoop(jValue * 0.5);
			manualAdjustment = 50;
			encoderState = 0;
		}
		else if(jValue < 0)
		{
			setOpenLoop(jValue * 0.5);
			manualAdjustment = 0;
			encoderState = 0;
		}
		else
		{
			switch(encoderState)
			{
				case 0:
					manualEncoderValue = encoderValue + manualAdjustment;
					encoderState = 1;
					break;
				case 1:
					setPosition(manualEncoderValue);
					break;
			}
		}
    }
    
    public static void setOpenLoop(double power)
	{
		shoppingCartSRX.set(ControlMode.PercentOutput, power);

    }
    
    static void deployShoppingCart()
    {
        setPosition(Constants.shoppingCartDeployed);
    }
    public static void moveToStowed()
    {
        setPosition(0);
    }
    public static void setPosition(int position)
	{
		shoppingCartSRX.set(ControlMode.MotionMagic, position);
	}
	//---------------------------------------------------------

	public static boolean positionThreshold(double constant)
	{
		if((constant + Constants.kWristPositionThreshold) > encoderValue && (constant - Constants.kWristPositionThreshold) < encoderValue)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// Encoder methods------------------------------------------
	public static void updateEncoder()
	{
		encoderValue = shoppingCartSRX.getSelectedSensorPosition(0);
		encoderVelocity = shoppingCartSRX.getSelectedSensorVelocity(0);
		encoderError = shoppingCartSRX.getClosedLoopError(0);
	}

	public static void setEncoderValue(int encoderValue)
	{
		shoppingCartSRX.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	//----------------------------------------------------------

	public static void resetEncoder()
	{
		shoppingCartSRX.setSelectedSensorPosition(0, Constants.kHatchWristPID, Constants.kTimeoutMs);
		encoderValue = 0;
    }
    
    public static void printPosition()
	{
		System.out.println("Encoder Value: " + encoderValue);
	}

	public static void stopWrist()
	{
		shoppingCartSRX.stopMotor();
	}
}