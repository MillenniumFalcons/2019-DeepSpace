package frc.team3647autonomous.PathProperties;

/**
 * every path property must have a string to be able to retrieve from the
 * roboRio's file system
 */
public abstract class PathProperties {
    public String asString;

    /**
     * every path property must have a string to be able to retrieve from the
     * roboRio's file system
     * @param name the name in the filename from path weaver
     */
    protected PathProperties(String name) {
        this.asString = name;
    }
}