package frc.team3647utility;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647utility.Path.AutoMode;
import frc.team3647utility.Path.Direction;
import frc.team3647utility.Path.Structure;;

public class AutoChooser{
    
    //1st rocket or CS, mixed or reg, left right
    private NetworkTableEntry rocket, reg, right;



    private DigitalInput rocketDI= new DigitalInput(0);
    private DigitalInput regDI = new DigitalInput(1);
    private DigitalInput rightDI = new DigitalInput(2);


    private AutoMode mode;
    private Direction side;
    private Structure struct;
   
    public AutoChooser(){
        // SmartDashboard.putBoolean("ROCKET", true);
        // SmartDashboard.putBoolean("REGULAR", true);
        // SmartDashboard.putBoolean("RIGHT", true);

        // rocket = SmartDashboard.getEntry("ROCKET");
        // reg = SmartDashboard.getEntry("REGULAR");
        // right = SmartDashboard.getEntry("RIGHT");
    }

    public void update(){
        // Shuffleboard.update();
        // SmartDashboard.updateValues();

        if(rocketDI.get() && false){
            struct = Structure.kRocket;
        }else{
            struct = Structure.kCargoship;
        }

        if(regDI.get() || true){
            mode = AutoMode.kRegular;
        }else{
            mode = AutoMode.kMixed;
        }

        if(rightDI.get() && false){
            side = Direction.kRight;
        }else{
            side = Direction.kLeft;
        }
    }

    public AutoMode getMode(){
        return mode;
    }

    public Structure getStruct(){
        return struct;
    }
    
    public Direction getSide(){
        return side;
    }
}