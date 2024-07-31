package POPLib.Subsytems;

import edu.wpi.first.wpilibj2.command.Command;

public interface StateMachineSubsytem<State> {
    public Command stablizeToNewState(State newState);
    public Command transitionState(State newState);

    public boolean isStable();
} 
