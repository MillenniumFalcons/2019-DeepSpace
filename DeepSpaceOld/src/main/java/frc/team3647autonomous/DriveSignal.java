package frc.team3647autonomous;

public class DriveSignal 
{

    private double left, right;

    public DriveSignal() 
    {
        this.left = 0;
        this.right = 0;
    }

    public DriveSignal(double left, double right) 
    {
        this.left = left;
        this.right = right;
    }

    public double getLeft() 
    {
        return left;
    }

    public double getRight() 
    {
        return right;
    }

    public void setLeft(double left) 
    {
        this.left = left;
    }

    public void setRight(double right) 
    {
        this.right = right;
    }
}
