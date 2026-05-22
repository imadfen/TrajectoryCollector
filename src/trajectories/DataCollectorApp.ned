package trajectories;

// Import the specific Veins 5.2 path
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple DataCollectorApp extends DemoBaseApplLayer
{
    parameters:
        @class(DataCollectorApp);
        @display("i=block/app");
        // Path to the decisions.json file produced by the Python pipeline
        // (src/deploy/export_decisions.py).  If the file does not exist the
        // simulation falls back to flat beaconing (baseline mode).
        string decisionsFile = default("decisions.json");
}