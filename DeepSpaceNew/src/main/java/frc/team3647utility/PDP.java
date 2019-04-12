/*----------------------------------------------------------------------------*/
/* Copyright (c) 2014-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3647utility;

import edu.wpi.first.hal.PDPJNI;

/**
 * Class for getting voltage, current, temperature, power and energy from the Power Distribution
 * Panel over CAN.
 */
public class PDP{
  private final int m_handle;

  /**
   * Constructor.
   *
   * @param module The CAN ID of the PDP
   */
  public PDP(int module) {
    m_handle = PDPJNI.initializePDP(module);
  }

  /**
   * Constructor.  Uses the default CAN ID (0).
   */
  public PDP() {
    this(0);
  }

  /**
   * Query the input voltage of the PDP.
   *
   * @return The voltage of the PDP in volts
   */
  public double getVoltage() {
    return PDPJNI.getPDPVoltage(m_handle);
  }

  /**
   * Query the temperature of the PDP.
   *
   * @return The temperature of the PDP in degrees Celsius
   */
  public double getTemperature() {
    return PDPJNI.getPDPTemperature(m_handle);
  }

  /**
   * Query the current of a single channel of the PDP.
   *
   * @return The current of one of the PDP channels (channels 0-15) in Amperes
   */
  public double getCurrent(int channel) {
    if(channel <=15 && channel >=0)
        return PDPJNI.getPDPChannelCurrent((byte) channel, m_handle);
    return -1.0;
  }

  /**
   * Query the current of all monitored PDP channels (0-15).
   *
   * @return The current of all the channels in Amperes
   */
  public double getTotalCurrent() {
    return PDPJNI.getPDPTotalCurrent(m_handle);
  }

  /**
   * Query the total power drawn from the monitored PDP channels.
   *
   * @return the total power in Watts
   */
  public double getTotalPower() {
    return PDPJNI.getPDPTotalPower(m_handle);
  }

  /**
   * Query the total energy drawn from the monitored PDP channels.
   *
   * @return the total energy in Joules
   */
  public double getTotalEnergy() {
    return PDPJNI.getPDPTotalEnergy(m_handle);
  }

  /**
   * Reset the total energy to 0.
   */
  public void resetTotalEnergy() {
    PDPJNI.resetPDPTotalEnergy(m_handle);
  }

  /**
   * Clear all PDP sticky faults.
   */
  public void clearStickyFaults() {
    PDPJNI.clearPDPStickyFaults(m_handle);
  }
}
