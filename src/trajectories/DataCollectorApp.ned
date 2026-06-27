package trajectories;

// Import the specific Veins 5.2 path
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple DataCollectorApp extends DemoBaseApplLayer
{
    parameters:
        @class(DataCollectorApp);
        @display("i=block/app");
        // Path to the decisions.json file 
        string decisionsFile = default("decisions.json");
}