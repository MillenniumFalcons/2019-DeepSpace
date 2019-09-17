package frc.team3647StateMachine.Movement;

import frc.team3647StateMachine.ElevatorLevel;

/**
 * requires a stop, or checkpoint at some value, that usually is minrotate
 */
public abstract class SafeMove extends Movement {
    public ElevatorLevel minRotateToUse = ElevatorLevel.MINROTATEHIGHER;

    /**
     * 
     * @param newLevel Which min rotate must the movement stop or start at
     */
    public void setMinRotateToUse(ElevatorLevel newLevel) {
        minRotateToUse = newLevel;
    }

}