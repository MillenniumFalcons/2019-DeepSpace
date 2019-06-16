package frc.team3647utility;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3647utility.Path.AutoMode;
import frc.team3647utility.Path.Direction;
import frc.team3647utility.Path.Structure;;

public class AutoChooser {

    // 1st rocket or CS, mixed or reg, left right
    private NetworkTableEntry rocket, reg, right;


    private AutoMode mode;
    private Direction side;
    private Structure struct;

    public AutoChooser() {
        SmartDashboard.putBoolean("ROCKET", true);
        SmartDashboard.putBoolean("REGULAR", true);
        SmartDashboard.putBoolean("RIGHT", true);

        rocket = SmartDashboard.getEntry("ROCKET");
        reg = SmartDashboard.getEntry("REGULAR");
        right = SmartDashboard.getEntry("RIGHT");
    }

    public void update() {
        Shuffleboard.update();
        SmartDashboard.updateValues();

        if (rocket.getBoolean(true)) {
            struct = Structure.kRocket;
        } else {
            struct = Structure.kCargoship;
        }

        if (reg.getBoolean(true)) {
            mode = AutoMode.kRegular;
        } else {
            mode = AutoMode.kMixed;
        }

        if (right.getBoolean(true)) {
            side = Direction.kRight;
        } else {
            side = Direction.kLeft;
        }
    }

    public AutoMode getMode() {
        return mode;
    }

    public Structure getStruct() {
        return struct;
    }

    public Direction getSide() {
        return side;
    }
}