/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3647subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Mop 
{
    private static Solenoid mopSolenoid = new Solenoid(Constants.mopSolenoidPin);

    public static void deployMop()
    {
        mopSolenoid.set(true);
    }

    public static void retractMop()
    {
        mopSolenoid.set(false);
    }

}
