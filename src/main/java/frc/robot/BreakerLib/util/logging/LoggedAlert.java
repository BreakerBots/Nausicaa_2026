package frc.robot.BreakerLib.util.logging;

import edu.wpi.first.wpilibj.Alert;

/**
 * An {@link Alert} that logs its state to BreakerLog whenever it is set or its text changes.
 * Use this when you want alerts visible on the dashboard and also recorded in log files for
 * post-match analysis (e.g. fault conditions, mode changes, operator warnings).
 */
public class LoggedAlert extends Alert {
    private String logKey;
    
    /**
   * Creates a new alert in the default group - "Alerts". If this is the first to be instantiated,
   * the appropriate entries will be added to NetworkTables.
   *
   * @param text Text to be displayed when the alert is active.
   * @param type Alert urgency level.
   */
  public LoggedAlert(String logKey, String text, AlertType type) {
    super(text, type);
    this.logKey = logKey;
    log();
  }

  /**
   * Creates a new alert. If this is the first to be instantiated in its group, the appropriate
   * entries will be added to NetworkTables.
   *
   * @param group Group identifier, used as the entry name in NetworkTables.
   * @param text Text to be displayed when the alert is active.
   * @param type Alert urgency level.
   */
  @SuppressWarnings("this-escape")
  public LoggedAlert(String logKey, String group, String text, AlertType type) {
    super(group, text, type);
    this.logKey = logKey;
    log();
  }

  @Override
  public void set(boolean active) {
      super.set(active);
      log();
  }

  @Override
  public void setText(String text) {
      super.setText(text);
      log();
  }

  public void log() {
    BreakerLog.log(logKey, this);
  }

}
