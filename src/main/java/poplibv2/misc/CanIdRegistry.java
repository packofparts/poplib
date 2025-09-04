package poplibv2.misc;

import java.util.HashSet;

import edu.wpi.first.wpilibj.DriverStation;

public class CanIdRegistry {
    static CanIdRegistry registry = null;
    HashSet<Integer> set; 

    public static CanIdRegistry getRegistry() {
        if (registry == null) {
            registry = new CanIdRegistry();
        }
        return registry;
    }

    public CanIdRegistry() {
        set = new HashSet<>();
    }

    public void registerCanId(int id) {
        int firstLength = set.size();
        set.add(id);
        if (set.size() == firstLength) {
            ErrorHandling.complainAboutDuplicatedCANIDException("Duplicate Can Id number " + id + " was used. Duplicate Can Ids lead to confusion during debugging and possibly errors. Dont use them.");
        }
    }
}
