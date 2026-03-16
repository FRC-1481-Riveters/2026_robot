// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftEnum;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import org.littletonrobotics.junction.Logger;

public class HubCounter extends SubsystemBase {

  private final BooleanPublisher isExternalPublisher;
  private final StringPublisher colorPublisher;
  private final IntegerPublisher patternPublisher;
  private final BooleanPublisher resetPublisher;
  private final BooleanPublisher pausePublisher;
  private final IntegerSubscriber countSubscriber;
  private final IntegerSubscriber pausedCountSubscriber;
  private final String blue = "#00c8ff";
  private final String red = "#ff0000";
  private final String black = "#000000";
  private final Double scoringDelay = 3.0;
  private final Double activeShiftEndingIndicator = 3.0;
  private boolean isExternal = true; // Default to true
  private boolean isScoringPaused = false; // Default to not paused

  public HubCounter() {
    var hcTable = NetworkTableInstance.getDefault().getTable("HubCounter");
    colorPublisher = hcTable.getStringTopic("Led/Color").publish();
    patternPublisher = hcTable.getIntegerTopic("Led/Pattern").publish();
    isExternalPublisher = hcTable.getBooleanTopic("IsExternal").publish();

    // Subscribe to the Count in Network Tables to get the current count
    countSubscriber = hcTable.getIntegerTopic("TotalCount").subscribe(0);
    pausedCountSubscriber = hcTable.getIntegerTopic("PausedTotalCount").subscribe(0);

    // Publish the Reset and Pause : True/False
    resetPublisher = hcTable.getBooleanTopic("ResetCounts").publish();
    pausePublisher = hcTable.getBooleanTopic("PauseCounting").publish();
  }

  public void initialize() {
    resetPublisher.set(true);
  }

  public void setExternal(boolean val) {
    isExternal = val;
  }

  // In the pattern publisher 0 is a solid patter, 1 is a pulsing pattern, and 2 in a chase/race
  // pattern
  public enum PatternEnum {
    SOLID(0),
    PULSE(1),
    CHASE(2);

    private final int value;

    PatternEnum(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  @Override
  public void periodic() {

    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isEmpty()) {
      return;
    }

    ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();

    boolean shiftActive = shiftInfo.active();

    // -- Set if scoring is enabled
    if (shiftActive || (!shiftActive && shiftInfo.elapsedTime() <= scoringDelay)) {
      isScoringPaused = false;
    } else {
      isScoringPaused = true;
    }

    // -- Set color
    String hubColor = (allianceColor.get() == Alliance.Blue) ? blue : red;
    if (!shiftInfo.active() || shiftInfo.remainingTime() <= 0) {
      hubColor = black;
    }

    // -- Set pattern
    PatternEnum hubPattern = PatternEnum.SOLID;
    if (shiftInfo.remainingTime() <= activeShiftEndingIndicator && shiftInfo.active()) {
      hubPattern = PatternEnum.PULSE;
    } else if (HubShiftUtil.getFirstActiveAlliance() != allianceColor.get()
        && shiftInfo.currentShift() == ShiftEnum.TRANSITION) {
      hubPattern = PatternEnum.CHASE;
    } else {
      hubPattern = PatternEnum.SOLID;
    }

    // -- Get values from Hub
    long succesfullyScoredFuel = countSubscriber.get();
    long unsuccesfullyScoredFuel = pausedCountSubscriber.get();

    // -- Publish configuration
    isExternalPublisher.set(isExternal);
    patternPublisher.set(hubPattern.getValue());
    colorPublisher.set(hubColor);
    pausePublisher.set(isScoringPaused);

    // -- Log Pattern and Color
    Logger.recordOutput("HubCounter/Control/Pattern", hubPattern.toString());
    Logger.recordOutput("HubCounter/Control/Color", hubColor);

    // -- Log Fuel related values
    Logger.recordOutput("HubCounter/ScoredFuel", succesfullyScoredFuel);
    Logger.recordOutput("HubCounter/ScoredFuelPaused", unsuccesfullyScoredFuel);
    Logger.recordOutput("HubCounter/Control/ScoringPaused", isScoringPaused);
  }
}
