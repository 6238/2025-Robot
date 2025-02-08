// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.BuildConstants;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/** This class logs pertinent metadata to NetworkTables. */
public class Logging {
  /** Private constructor because this should never be instantiated. */
  private Logging() {}
  ;

  // https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/METADATA.md
  // Data put in the `/Metadata/` table as strings is hidden in AdvantageScope by default,
  // except in the "Metadata" tab.
  /** Logs metadata to NetworkTables. */
  @SuppressWarnings("all")
  public static void logMetadata() {
    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/BuildDate")
        .publish()
        .set(BuildConstants.BUILD_DATE);

    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/GitCommitDate")
        .publish()
        .set(BuildConstants.GIT_DATE);

    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/GitCommitHash")
        .publish()
        .set(BuildConstants.GIT_SHA);

    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/GitDirty")
        .publish()
        .set(
            BuildConstants.DIRTY == 1
                ? "✏️ Uncommitted changes present"
                : "✅ All changes committed");

    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/GitBranch")
        .publish()
        .set(BuildConstants.GIT_BRANCH);

    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/RuntimeType")
        .publish()
        .set(RobotBase.getRuntimeType().toString());

    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/ProjectName")
        .publish()
        .set(BuildConstants.MAVEN_NAME);
  }

  public static void initializeCommandSchedulerHooks() {
    CommandScheduler.getInstance().onCommandInitialize(initializeHook());
    CommandScheduler.getInstance().onCommandFinish(finishHook());
    CommandScheduler.getInstance().onCommandInterrupt(initializeHook());
  }

  private static Consumer<Command> initializeHook() {
    return (Command command) -> {
      DataLogManager.log("INITIALIZE: " + command.getName());
    };
  }

  private static Consumer<Command> finishHook() {
    return (Command command) -> {
      DataLogManager.log("FINISH: " + command.getName());
    };
  }

  private static BiConsumer<Command, Optional<Command>> interruptHook() {
    return (Command command, Optional<Command> other) -> {
      if (other.isPresent()) {
        DataLogManager.log("INTERRUPT: " + command.getName() + " by " + other.get().getName());
        return;
      }

      DataLogManager.log("INTERRUPT: " + command.getName());
    };
  }
}
