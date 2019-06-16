package frc.team3647subsystems;

public abstract class Subsystem {

    public abstract void setOpenLoop(double demand);

    public void stop() {
        setOpenLoop(0);
    }
}