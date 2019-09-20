package frc.team3647subsystems;

/**
 * any subsystem that includes a motor
 */
public abstract class Subsystem {

    /**
     * 
     * @param demand power to send to motor
     */
    public abstract void setOpenLoop(double demand);

    /**
     * sets open loop to 0
    */
    public void stop() {
        setOpenLoop(0);
    }
}