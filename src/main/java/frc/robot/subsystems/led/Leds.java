// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Optional;

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean hpAttentionAlert = false;
  public boolean hasAlge = false;
  public boolean endgameAlert = false;
  public boolean autoScoringReef = false;
  public boolean autoScoring = false;
  public boolean superstructureCoast = false;
  public boolean superstructureEstopped = false;
  public boolean lowBatteryAlert = false;
  public boolean characterizationMode = false;
  public boolean visionDisconnected = false;
  public boolean climbing = false;
  public boolean coralGrabbed = false;
  public boolean firstPriorityBlocked = false;
  public boolean secondPriorityBlocked = false;
  public Color hexColor = Color.kBlack;
  public Color secondaryHexColor = Color.kBlack;
  public boolean hpAttentionLeftAlert = false;
  public boolean hpAttentionRightAlert = false;

  private Optional<Alliance> alliance = Optional.empty();
  private Color disabledColor = Color.kDarkRed;
  private Color secondaryDisabledColor = Color.kWhite;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 30;
  private static final Section fullSection = new Section(0, length);
  private static final Section rightSection = new Section(length / 2, length);
  private static final Section leftSection = new Section(0, length / 2);
  private static final Section topThreeQuartSection = new Section(length / 4, length);
  private static final Section bottomQuartSection = new Section(0, length / 4);
  private static final double strobeDuration = 0.1;
  private static final double breathFastDuration = 0.5;
  private static final double breathSlowDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveDisabledCycleLength = 15.0;
  private static final double waveDisabledDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  private Leds() {
    leds = new AddressableLED(5);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    fullSection,
                    Color.kWhite,
                    Color.kBlack,
                    breathSlowDuration,
                    System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      disabledColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(disabledColor);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : secondaryDisabledColor;
    }

    // Update auto state
    if (DriverStation.isEnabled()) {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getTimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    solid(fullSection, Color.kBlack); // Default to off
    if (estopped) {
      solid(fullSection, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (superstructureCoast) {
        // Elevator coast alert
        solid(fullSection, Color.kWhite);
      } else if (lastEnabledAuto && Timer.getTimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        wave(
            new Section(
                0,
                (int) (length * (1 - ((Timer.getTimestamp() - lastEnabledTime) / autoFadeTime)))),
            Color.kGold,
            Color.kDarkBlue,
            waveFastCycleLength,
            waveFastDuration);
      } else if (lowBatteryAlert) {
        // Low battery
        strobe(fullSection, Color.kOrangeRed, Color.kBlack, strobeDuration);
      } else if (prideLeds) {
        // Pride stripes
        stripes(
            fullSection,
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            3,
            5.0);
      } else {
        // Default pattern
        wave(
            fullSection,
            disabledColor,
            secondaryDisabledColor,
            waveDisabledCycleLength,
            waveDisabledDuration);
      }

      // Vision disconnected alert
      if (visionDisconnected) {
        strobe(bottomQuartSection, Color.kRed, Color.kBlack, strobeDuration);
      }

    } else if (DriverStation.isAutonomous()) {
      if (characterizationMode) {
        strobe(fullSection, Color.kGold, Color.kBlack, 0.5);
      } else {
        wave(fullSection, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
      }
    } else {
      solid(rightSection, hexColor);
      solid(leftSection, secondaryHexColor);

      // Auto scoring
      if (autoScoring) {
        rainbow(fullSection, rainbowCycleLength, rainbowDuration);
      }

      // Auto scoring
      if (hasAlge) {
        solid(fullSection, Color.kAliceBlue);
      }

      // Climbing alert
      if (climbing) {
        strobe(fullSection, Color.kGold, Color.kDarkBlue, strobeDuration);
      }
      // Climbing alert
      if (hpAttentionLeftAlert) {
        strobe(fullSection, Color.kGreen, Color.kWhite, 0.5);
      }
      // Climbing alert
      if (hpAttentionRightAlert) {
        strobe(fullSection, Color.kWhite, Color.kGreen, 0.5);
      }

      // Coral grab alert
      if (coralGrabbed) {
        solid(fullSection, Color.kLime);
      }

      // Human player alert
      if (hpAttentionAlert) {
        strobe(fullSection, Color.kWhite, Color.kBlack, strobeDuration);
      }

      // Endgame alert
      if (endgameAlert) {
        strobe(fullSection, Color.kRed, Color.kGold, strobeDuration);
      }
    }

    // Superstructure estop alert
    if (superstructureEstopped) {
      solid(fullSection, Color.kRed);
    }

    // Update dashboard
    SmartDashboard.putString("LEDs/First Priority", hexColor.toHexString());
    SmartDashboard.putString("LEDs/Second Priority", secondaryHexColor.toHexString());

    // Update LEDs
    leds.setData(buffer);

    // Record cycle time
    LoggedTracer.record("LEDs");
  }

  private Color solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
    return color;
  }

  private Color strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    return solid(section, c1On ? c1 : c2);
  }

  private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    solid(section, color);
    return color;
  }

  private Color breathCalculate(Section section, Color c1, Color c2, double duration) {
    double x = ((Timer.getTimestamp() % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    return color;
  }

  private Color breath(Section section, Color c1, Color c2, double duration) {
    return breath(section, c1, c2, duration, Timer.getTimestamp());
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private static record Section(int start, int end) {}
}
