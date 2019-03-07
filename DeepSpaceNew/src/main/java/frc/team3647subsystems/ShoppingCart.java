/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3647subsystems;


import frc.robot.*;
import frc.team3647inputs.Joysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/**
 * Add your docs here.
 */
public class ShoppingCart 
{
	public static WPI_TalonSRX shoppingCartMotor = new WPI_TalonSRX(Constants.shoppingCartMotorPin);
	private static VictorSPX shoppingCartSPX = new VictorSPX(Constants.shoppinCartSPXPin); // motor for shopping cart wheels
    public static ShoppingCartPosition currentState, aimedState;
    public static int shoppingCartEncoderCCL, shoppingCartEncoderValue, shoppingCartEncoderVelocity;
	public static double overrideValue;
	public static boolean manualOverride;

    public static void shoppingCartInit()
	{
		//Motor & Sensor Direction
		shoppingCartMotor.setInverted(false);
		shoppingCartMotor.setSensorPhase(true);
		
		//Current Limiting
		shoppingCartMotor.configPeakCurrentLimit(Constants.kHatchWristPeakCurrent, Constants.kTimeoutMs);
		shoppingCartMotor.configPeakCurrentDuration(Constants.kHatchWristPeakCurrentDuration);
		shoppingCartMotor.configContinuousCurrentLimit(Constants.kHatchWristContinuousCurrent, Constants.kTimeoutMs);
		shoppingCartMotor.enableCurrentLimit(true);
		//PID
		shoppingCartMotor.config_kP(Constants.kHatchWristPID, Constants.kHatchWristP, Constants.kTimeoutMs);
		shoppingCartMotor.config_kI(Constants.kHatchWristPID, Constants.kHatchWristI, Constants.kTimeoutMs);
		shoppingCartMotor.config_kD(Constants.kHatchWristPID, Constants.kHatchWristD, Constants.kTimeoutMs);
		shoppingCartMotor.config_kF(Constants.kHatchWristPID, Constants.kHatchWristF, Constants.kTimeoutMs);

		//Motion Magic Constants
		shoppingCartMotor.configMotionAcceleration(Constants.kHatchWristAcceleration, Constants.kTimeoutMs);
        shoppingCartMotor.configMotionCruiseVelocity(Constants.kHatchWristCruiseVelocity, Constants.kTimeoutMs);
        resetEncoder();
    }

    public void configPIDFVA(double p, double i, double d, double f, int vel, int accel) //for configuring PID values
	{
		//PID
		shoppingCartMotor.config_kP(Constants.kHatchWristPID, p, Constants.kTimeoutMs);
		shoppingCartMotor.config_kI(Constants.kHatchWristPID, i, Constants.kTimeoutMs);
		shoppingCartMotor.config_kD(Constants.kHatchWristPID, d, Constants.kTimeoutMs);
		shoppingCartMotor.config_kF(Constants.kHatchWristPID, f, Constants.kTimeoutMs);
		
		// motion magic
		shoppingCartMotor.configMotionAcceleration(accel, Constants.kTimeoutMs);
		shoppingCartMotor.configMotionCruiseVelocity(vel, Constants.kTimeoutMs);

	}
    
    public static enum ShoppingCartPosition
	{
		STOWED,
		DEPLOYED,
		MANUAL
    }

    public static void runShoppingCart()
    {
        setShoppinCartEncoder();
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
            }
        }
    }
    
	public static void runShoppingCartSPX(double demand)
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
		setShoppinCartEncoder();
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
					manualEncoderValue = shoppingCartEncoderValue + manualAdjustment;
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
		shoppingCartMotor.set(ControlMode.PercentOutput, power);

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
		shoppingCartMotor.set(ControlMode.MotionMagic, position);
	}
	//---------------------------------------------------------

	public static boolean positionThreshold(double constant)
	{
		if((constant + Constants.kWristPositionThreshold) > shoppingCartEncoderValue && (constant - Constants.kWristPositionThreshold) < shoppingCartEncoderValue)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// Encoder methods------------------------------------------
	public static void setShoppinCartEncoder()
	{
		shoppingCartEncoderValue = shoppingCartMotor.getSelectedSensorPosition(0);
		shoppingCartEncoderVelocity = shoppingCartMotor.getSelectedSensorVelocity(0);
		shoppingCartEncoderCCL = shoppingCartMotor.getClosedLoopError(0);
	}

	public static void setEncoderValue(int encoderValue)
	{
		shoppingCartMotor.setSelectedSensorPosition(encoderValue, 0, Constants.kTimeoutMs);
	}
	//----------------------------------------------------------

	public static void resetEncoder()
	{
		shoppingCartMotor.setSelectedSensorPosition(0, Constants.kHatchWristPID, Constants.kTimeoutMs);
		shoppingCartEncoderValue = 0;
    }
    
    public static void printPosition()
	{
		// System.out.println("Encoder Value: " + shoppingCartEncoderValue);
	}

	public static void stopWrist()
	{
		shoppingCartMotor.stopMotor();
	}
}