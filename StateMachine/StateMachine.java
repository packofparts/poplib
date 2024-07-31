package POPLib.StateMachine;

import java.util.HashMap;
import java.util.Map;
import POPLib.Subsytems.StateMachineSubsytem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class StateMachine<State> {
    private State currentState;
    private Map<State, StateMachineSubsytem<State>[]> stateToSubsytem;

    public StateMachine(State currentState, StateMachineSubsytem<State> ... subsytems) {
        this.currentState = currentState;

        this.stateToSubsytem = new HashMap<State, StateMachineSubsytem<State>[]>();

        // for ()
    }

    // public void registerState(State state, StateMachineSubsytem<State> ... subsytems) {
    //     this.stateToSubsytem.put(currentState, subsytems);
    // }

    // public Command transitionState(State newState) {
    //     Command ret = new InstantCommand(() -> {
    //         System.out.println("Transitioning State To: " + newState.toString() + " from state: " + currentState.toString());
    //         currentState = newState;
    //     });

    //     for (StateMachineSubsytem<State> i : subsytems) {
    //         ret = ret.alongWith(i.transitionState(newState));
    //     }

    //     return ret;
    // }

    // private Boolean robotStable() {
    //     for (StateMachineSubsytem<State> i : subsytems) {
    //         if (!i.isStable()) {
    //             return false;
    //         }
    //     }

    //     return true;
    // }

    // public void logState() {
    //     SmartDashboard.putString("State ", currentState.toString());
    // }
}
