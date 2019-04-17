package frc.team3647utility;

import frc.team3647autonomous.AutonomousSequences;

public class Path
{

    private Direction direction;
    private Structure structure;
    private AutoMode autoMode;
    private String intialPath;

    private Runnable method;

    public enum Direction
    {
        kRight("Right"),
        kLeft("Left"),
        kMiddle("Middle");

        public String str;
        Direction(String str){
            this.str = str;
        }
    }

    public enum Structure
    {
        kRocket("Rocket"),
        kCargoship("CargoShip");

        public String str;
        Structure(String str){
            this.str = str;
        }
    }

    public enum AutoMode
    {
        kRegular("Front"),
        kMixed("Back");

        public String str;
        AutoMode(String str){
            this.str = str;
        }
    }

    public Path(Direction direction, Structure structure, AutoMode autoMode)
    {
        this.direction = direction;
        this.structure = structure;
        this.autoMode = autoMode;
        this.intialPath = buildInitialPathString(direction, structure, autoMode);
        updateMethod();        
    }

    public void update(Direction direction, Structure structure, AutoMode autoMode)
    {
        this.direction = direction;
        this.structure = structure;
        this.autoMode = autoMode;
        this.intialPath = buildInitialPathString(direction, structure, autoMode);
        updateMethod();
    }

    private void updateMethod(){
        if(structure.equals(Structure.kRocket))
        {
            if(autoMode.equals(AutoMode.kRegular)){
                method = () -> {AutonomousSequences.frontRocketAuto(direction.str);};
            }else{
                method = () -> {AutonomousSequences.mixedRocketAuto(direction.str);};
            }
        }
        else
        {
            if(autoMode.equals(AutoMode.kRegular)){
                method = () -> {AutonomousSequences.sideCargoShipAuto(direction.str);};
            }else{
                method = () -> {AutonomousSequences.mixedCargoShipAuto(direction.str);};
            }
        }
    }

    public String getDirection(Direction direction)
    {
        return direction.str;
    }

    public String getStructure(Structure structure)
    {
       return structure.str;
    }

    public String getAutoMode(AutoMode autoMode)
    {
        return autoMode.str;
    }

    private String buildInitialPathString(Direction direction, Structure structure, AutoMode autoMode)
    {
        String toReturn = direction.str + "Platform2To" + direction.str + structure.str;

        if(autoMode.equals(AutoMode.kRegular) && structure.equals(Structure.kCargoship))
            toReturn += "Bay1";
            
        return toReturn;
    }

    public String getIntialPath()
    {
        return this.intialPath;
    }

    public void run(){
        method.run();
    }
}