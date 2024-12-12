import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

interface Action {
    boolean run();
}

class SequentialAction implements Action {
    private final List<Action> actions;

    public SequentialAction(Action... actions) {
        this.actions = Arrays.asList(actions);
    }

    @Override
    public boolean run() {
        for (Action action : actions) {
            if (!action.run()) {
                return false;
            }
        }
        return true;
    }
}
class ParallelAction implements Action {
    private final List<Action> actions;

    public ParallelAction(Action... actions) {
        this.actions = Arrays.asList(actions);
    }

    @Override
    public boolean run() {
        boolean allCompleted = true;
        for (Action action : actions) {
            allCompleted &= action.run();
        }
        return allCompleted;
    }
}

class LiftFroggy {
    private int liftPosition = 0;

    public Action liftUp() {
        return () -> {
            if (liftPosition < 1000) {
                liftPosition += 100;
                System.out.println("Lifting up: Position=" + liftPosition);
                return false;
            }
            System.out.println("Lift up complete");
            return true;
        };
    }

    public Action liftBack() {
        return () -> {
            System.out.println("Moving lift back...");
            return true;
        };
    }
}

class PushFroggy {
    public Action pushSetup() {
        return () -> {
            System.out.println("Setting up push...");
            return true;
        };
    }
}

class TrajectoryAction implements Action {
    private final String trajectoryName;

    public TrajectoryAction(String trajectoryName) {
        this.trajectoryName = trajectoryName;
    }

    @Override
    public boolean run() {
        System.out.println("Running trajectory: " + trajectoryName);
        return true;
    }
}

public class frogtonomousred {
    public static void main(String[] args) {
        LiftFroggy liftFroggy = new LiftFroggy();
        PushFroggy pushFroggy = new PushFroggy();

        TrajectoryAction traj1 = new TrajectoryAction("Trajectory 1");
        TrajectoryAction traj2 = new TrajectoryAction("Trajectory 2");

        Action actionSequence = new SequentialAction(
                traj1,
                new ParallelAction(
                        liftFroggy.liftUp(),
                        () -> {
                            int customActionCounter = 0;
                            if (customActionCounter < 5) {
                                System.out.println("Executing custom lambda parallel action: Step " + (++customActionCounter));
                                return false;
                            }
                            System.out.println("Custom lambda parallel action complete");
                            return true;
                        }
                ),
                traj2,
                pushFroggy.pushSetup()
        );

        while (!actionSequence.run()) {
            System.out.println("Action sequence in progress...");
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        System.out.println("All actions completed successfully.");
    }
}
