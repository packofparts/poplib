package poplibv2.misc;

import java.util.HashSet;

public class CanIdRegistry {
    static CanIdRegistry registry = null;
    HashSet<Integer> set; 

    /**
     * POPLIB INTERNAL FUNCTION
     * <p></p>
     * Gets the CAN ID Registry
     * @return
     */
    public static CanIdRegistry getRegistry() {
        if (registry == null) {
            registry = new CanIdRegistry();
        }
        return registry;
    }

    /**
     * Creates a new registry.
     */
    private CanIdRegistry() {
        set = new HashSet<>();
    }

    /**
     * Registers a new CAN id to the registry.
     * If the id is already in the registry, to outputs an error in driverstation that complains about dupicate can ids.
     * @param id
     */
    public void registerCanId(int id) {
        int firstLength = set.size();
        set.add(id);
        if (set.size() == firstLength) {
            ErrorHandling.complainAboutDuplicatedCANIDException("Duplicate Can Id number " + id + " was used. Duplicate Can Ids lead to confusion during debugging and possibly errors. Dont use them.");
        }
    }
}
