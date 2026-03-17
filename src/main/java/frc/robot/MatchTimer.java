package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public final class MatchTimer {
    private MatchTimer() {}

    // When we're not connected to an FMS, we simulate
    private static double simStartTime = -1.0;
    private static double simDuration = 160.0;

    private static final double THROTTLE_INTERVAL_SEC = 0.5;
    private static double lastPublishTime = -1.0;

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

        // Match time is -1.0 when not connected to FMS
        if (matchTime < 0.0 && DriverStation.isEnabled()) {
            if (simStartTime < 0) {
                simStartTime = Timer.getFPGATimestamp();
                simDuration = DriverStation.isAutonomous() ? 20.0 : 140.0;
            }
            matchTime = Math.max(0, simDuration - (Timer.getFPGATimestamp() - simStartTime));
        } else if (!DriverStation.isEnabled()) {
            simStartTime = -1.0;
        }

        boolean validTime = matchTime >= 0.0;
        int displaySeconds = validTime ? (int) Math.round(matchTime) : 0;

        MatchPhase phase = getMatchPhase(matchTime);
        String phaseLabel = getDriverLabel(phase);
        String transitionWarning = getTransitionWarning(phase, displaySeconds);

        // Publish directly to NetworkTables so dashboard works when FMS is connected
        // (BreakerLog disables NT publishing during matches to conserve bandwidth)
        double now = Timer.getFPGATimestamp();
        if (lastPublishTime < 0 || now - lastPublishTime >= THROTTLE_INTERVAL_SEC) {
            lastPublishTime = now;
            var table = NetworkTableInstance.getDefault().getTable("Robot").getSubTable("MatchTimer");
            table.getEntry("Time").setDouble(displaySeconds);
            table.getEntry("Phase").setString(phaseLabel);
            table.getEntry("matchTime").setDouble(matchTime);
            table.getEntry("TransitionWarning").setString(transitionWarning);
        }
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

    /** Always returns "[NEXT_PHASE] in [x]s!" — uses displaySeconds so it matches the Time display. */
    public static String getTransitionWarning(MatchPhase phase, int displaySeconds) {
        if (phase == MatchPhase.DISABLED || displaySeconds < 0) return "";

        String nextPhaseLabel;
        int secondsUntil;
        switch (phase) {
            case AUTO -> { nextPhaseLabel = "TRANSITION"; secondsUntil = displaySeconds; }
            case TRANSITION -> { nextPhaseLabel = "SHIFT 1"; secondsUntil = displaySeconds - 130; }
            case SHIFT_1 -> { nextPhaseLabel = "SHIFT 2"; secondsUntil = displaySeconds - 105; }
            case SHIFT_2 -> { nextPhaseLabel = "SHIFT 3"; secondsUntil = displaySeconds - 80; }
            case SHIFT_3 -> { nextPhaseLabel = "SHIFT 4"; secondsUntil = displaySeconds - 55; }
            case SHIFT_4 -> { nextPhaseLabel = "ENDGAME"; secondsUntil = displaySeconds - 30; }
            case ENDGAME -> { nextPhaseLabel = "Match end"; secondsUntil = displaySeconds; }
            default -> { return ""; }
        }
        int sec = Math.max(0, secondsUntil);
        return nextPhaseLabel + " in " + sec + "s!";
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
