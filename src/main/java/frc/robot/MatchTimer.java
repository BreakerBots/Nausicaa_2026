package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public final class MatchTimer {
    private MatchTimer() {}

    public enum MatchPhase {
        DISABLED,
        AUTO,        // 20s
        TRANSITION,  // 10s
        SHIFT_1,     // 25s
        SHIFT_2,     // 25s
        SHIFT_3,     // 25s
        SHIFT_4,     // 25s
        ENDGAME      // 30s
    }

    public static void update() {
        double matchTime = DriverStation.getMatchTime();

        // Match time can be negative or invalid when not connected / disabled.
        boolean validTime = matchTime >= 0.0;

        MatchPhase phase = getMatchPhase(matchTime);

        BreakerLog.log("MatchTimer/Time", validTime ? matchTime : 0.0);
        BreakerLog.log("MatchTimer/Phase", getDriverLabel(phase));
        BreakerLog.log("MatchTimer/TransitionWarning", getTransitionWarning(phase, matchTime));
    }

    public static MatchPhase getMatchPhase(double matchTime) {

        if (!DriverStation.isEnabled()) {
            return MatchPhase.DISABLED;
        }

        if (DriverStation.isAutonomous()) {
            return MatchPhase.AUTO;
        }

        if (matchTime > 130.0) {
            return MatchPhase.TRANSITION;
        } else if (matchTime > 105.0) {
            return MatchPhase.SHIFT_1;
        } else if (matchTime > 80.0) {
            return MatchPhase.SHIFT_2;
        } else if (matchTime > 55.0) {
            return MatchPhase.SHIFT_3;
        } else if (matchTime > 30.0) {
            return MatchPhase.SHIFT_4;
        } else if (matchTime >= 0.0) {
            return MatchPhase.ENDGAME;
        } else {
            return MatchPhase.DISABLED;
        }
    }

    /** Returns "Transition in [x]s" when within 5s of next phase, else "". */
    public static String getTransitionWarning(MatchPhase phase, double matchTime) {
        double secondsUntilTransition = -1.0;
        switch (phase) {
            case TRANSITION -> { if (matchTime <= 125.0 && matchTime > 120.0) secondsUntilTransition = matchTime - 120.0; }
            case SHIFT_1 -> { if (matchTime <= 110.0 && matchTime > 105.0) secondsUntilTransition = matchTime - 105.0; }
            case SHIFT_2 -> { if (matchTime <= 85.0 && matchTime > 80.0) secondsUntilTransition = matchTime - 80.0; }
            case SHIFT_3 -> { if (matchTime <= 60.0 && matchTime > 55.0) secondsUntilTransition = matchTime - 55.0; }
            case SHIFT_4 -> { if (matchTime <= 35.0 && matchTime > 30.0) secondsUntilTransition = matchTime - 30.0; }
            case ENDGAME -> { if (matchTime <= 5.0 && matchTime >= 0.0) secondsUntilTransition = matchTime; }
            default -> {}
        }
        if (secondsUntilTransition < 0) return "";
        int sec = (int) Math.ceil(secondsUntilTransition);
        return "Transition in " + sec + "s";
    }

    public static String getDriverLabel(MatchPhase phase) {
        return switch (phase) {
            case AUTO -> "AUTO";
            case TRANSITION -> "TRANSITION";
            case SHIFT_1 -> "SHIFT 1";
            case SHIFT_2 -> "SHIFT 2";
            case SHIFT_3 -> "SHIFT 3";
            case SHIFT_4 -> "SHIFT 4";
            case ENDGAME -> "ENDGAME";
            case DISABLED -> "READY";
        };
    }
}
