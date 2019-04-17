package frc.team3647utility;


public class Path
{

    private Direction direction;
    private Structure structure;
    private AutoMode autoMode;
    private String intialPath;

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
    }

    public void update(Direction direction, Structure structure, AutoMode autoMode)
    {
        this.direction = direction;
        this.structure = structure;
        this.autoMode = autoMode;
        this.intialPath = buildInitialPathString(direction, structure, autoMode);

        
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
        String toReturn = "";
        switch(autoMode)
        {
            case kMixed:
                if(structure == Structure.kCargoship)
                    toReturn = "PlatformTo" + getDirection(direction) + "Middle" + getDirection(direction) + getStructure(structure);
                else
                    toReturn = "PlatformTo" + getDirection(direction) + getAutoMode(autoMode) + getStructure(structure);
                break;
            case kRegular:
                if(structure == Structure.kCargoship)
                    toReturn = "PlatformTo" + getDirection(direction) + getStructure(structure) + "Bay1";
                else
                    toReturn = "PlatformTo" + getDirection(direction) + getAutoMode(autoMode) + getStructure(structure);
                break;
            default:
                toReturn = "PlatformTo" + getDirection(direction) + getAutoMode(autoMode) + getStructure(structure);
                break;
        }
        return toReturn;
    }

    public String getIntialPath()
    {
        return this.intialPath;
    }

    
}