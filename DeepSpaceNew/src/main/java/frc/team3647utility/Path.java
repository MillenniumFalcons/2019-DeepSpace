package frc.team3647utility;


public class Path
{

    private Direction direction;
    private Structure structure;
    private AutoMode autoMode;
    private Runnable runner;
    private String intialPath;

    public enum Direction
    {
        kRight,
        kLeft,
        kMiddle;
    }

    public enum Structure
    {
        kRocket,
        kCargoship;
    }

    public enum AutoMode
    {
        kRegular,
        kMixed;
    }

    public Path(Direction direction, Structure structure, AutoMode autoMode)
    {
        this.direction = direction;
        this.structure = structure;
        this.autoMode = autoMode;
        this.intialPath = buildInitialPathString(direction, structure, autoMode);
    }

    public void run()
    {
        runner.run();
    }

    public String getDirection(Direction direction)
    {
        String toReturn = "";

        switch(direction)
        {
            case kLeft:
                toReturn = "Left";
                break;
            case kRight:
                toReturn = "Right";
                break;
            default:
                toReturn = "Middle";

        }

        return toReturn;
    }

    public String getStructure(Structure structure)
    {
        String toReturn = "";

        switch(structure)
        {
            case kCargoship:
                toReturn = "CargoShip";
                break;
            case kRocket:
                toReturn = "Rocket";
                break;
            default:
                toReturn = "";

        }

        return toReturn;
    }

    public String getAutoMode(AutoMode autoMode)
    {
        String toReturn = "";

        switch(autoMode)
        {
            case kMixed:
                toReturn = "Back";
                break;
            case kRegular:
                toReturn = "Front";
                break;
            default:
                toReturn = "";

        }

        return toReturn;
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