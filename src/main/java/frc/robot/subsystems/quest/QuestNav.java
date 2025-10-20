package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.proto.Pose2dProto;
import edu.wpi.first.math.proto.Geometry2D;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.ProtobufSubscriber;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import gg.questnav.questnav.PoseFrame;
import frc.robot.generated.Commands;
import frc.robot.generated.Data;
import frc.robot.protos.wpilib.CommandProto;
import frc.robot.protos.wpilib.CommandResponseProto;
import frc.robot.protos.wpilib.DeviceDataProto;
import frc.robot.protos.wpilib.FrameDataProto;

import java.util.OptionalDouble;
import java.util.OptionalInt;

/**
 * The QuestNav class provides an interface to communicate with an Oculus/Meta Quest VR headset for
 * robot localization and tracking purposes. It uses NetworkTables to exchange data between the
 * robot and the Quest device.
 */
public class QuestNav {

  /** NetworkTable instance used for communication */
  private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();

  /** NetworkTable for Quest navigation data */
  private final NetworkTable questNavTable = nt4Instance.getTable("QuestNav");

  /** Protobuf instance for CommandResponse */
  private final CommandResponseProto commandResponseProto = new CommandResponseProto();

  /** Protobuf instance for Command */
  private final CommandProto commandProto = new CommandProto();

  /** Protobuf instance for Pose2d */
  private final Pose2dProto pose2dProto = new Pose2dProto();

  /** Protobuf instance for device data */
  private final DeviceDataProto deviceDataProto = new DeviceDataProto();

  /** Protobuf instance for frame data */
  private final FrameDataProto frameDataProto = new FrameDataProto();

  /** Subscriber for command response */
  private final ProtobufSubscriber<Commands.ProtobufQuestNavCommandResponse> responseSubscriber =
      questNavTable
          .getProtobufTopic("response", commandResponseProto)
          .subscribe(Commands.ProtobufQuestNavCommandResponse.newInstance());

  /** Subscriber for frame data */
  private final ProtobufSubscriber<Data.ProtobufQuestNavFrameData> frameDataSubscriber =
      questNavTable
          .getProtobufTopic("frameData", frameDataProto)
          .subscribe(
              Data.ProtobufQuestNavFrameData.newInstance(),
              PubSubOption.periodic(0.01),
              PubSubOption.sendAll(true),
              PubSubOption.pollStorage(20));

  /** Subscriber for device data */
  private final ProtobufSubscriber<Data.ProtobufQuestNavDeviceData> deviceDataSubscriber =
      questNavTable
          .getProtobufTopic("deviceData", deviceDataProto)
          .subscribe(Data.ProtobufQuestNavDeviceData.newInstance());

  /** Publisher for command requests */
  private final ProtobufPublisher<Commands.ProtobufQuestNavCommand> requestPublisher =
      questNavTable.getProtobufTopic("request", commandProto).publish();

  /** Cached request to lessen GC pressure */
  private final Commands.ProtobufQuestNavCommand cachedCommandRequest =
      Commands.ProtobufQuestNavCommand.newInstance();

  /** Cached pose reset request to lessen GC pressure */
  private final Commands.ProtobufQuestNavPoseResetPayload cachedPoseResetPayload =
      Commands.ProtobufQuestNavPoseResetPayload.newInstance();

  /** Cached proto pose (for reset requests) to lessen GC pressure */
  private final Geometry2D.ProtobufPose2d cachedProtoPose = Geometry2D.ProtobufPose2d.newInstance();

  /** Last sent request id */
  private int lastSentRequestId = 0; // Should be the same on the backend

  /** Last processed response id */
  private int lastProcessedResponseId = 0; // Should be the same on the backend

  /** Pose of quest when pose was reset */
  private Pose2d uncorrectedResetPose = Pose2d.kZero;

  /** Heading of quest when pose was reset */
  private Rotation2d resetHeading = Rotation2d.kZero;

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = questNavTable.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
    questNavTable.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion =
    questNavTable.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles =
    questNavTable.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent =
    questNavTable.getDoubleTopic("device/batteryPercent").subscribe(0.0f);


  /** Creates a new QuestNav implementation */
  public QuestNav() {}

  /**
   * Sets the field-relative pose of the Quest. This is the position of the Quest, not the robot.
   * Make sure you correctly offset back from the center of your robot first.
   *
   * @param pose The field relative position of the Quest
   */
  public void setPose(Pose2d pose) {
    cachedProtoPose.clear(); // Clear instead of creating new
    pose2dProto.pack(cachedProtoPose, pose);
    cachedCommandRequest.clear();
    var requestToSend =
        cachedCommandRequest
            .setType(Commands.QuestNavCommandType.POSE_RESET)
            .setCommandId(++lastSentRequestId)
            .setPoseResetPayload(cachedPoseResetPayload.clear().setTargetPose(cachedProtoPose));

    requestPublisher.set(requestToSend);
  }

  /**
   * Returns the Quest's battery level (0-100%)
   *
   * @return An Optional containing battery percentage, or an empty Optional if no data is available
   */
  public OptionalInt getBatteryPercent() {
    Data.ProtobufQuestNavDeviceData latestDeviceData = deviceDataSubscriber.get();
    if (latestDeviceData != null) {
      return OptionalInt.of(latestDeviceData.getBatteryPercent());
    }
    return OptionalInt.empty();
  }

  /**
   * Gets the current tracking state of the Quest headset.
   *
   * @return Boolean indicating if the Quest is currently tracking (true) or not (false)
   */
  public boolean isTracking() {
    Data.ProtobufQuestNavDeviceData latestDeviceData = deviceDataSubscriber.get();
    if (latestDeviceData != null) {
      return latestDeviceData.getCurrentlyTracking();
    }
    return false; // Return false if no data for failsafe
  }

  /**
   * Gets the current frame count from the Quest headset.
   *
   * @return The frame count value
   */
  public OptionalInt getFrameCount() {
    Data.ProtobufQuestNavFrameData latestFrameData = frameDataSubscriber.get();
    if (latestFrameData != null) {
      return OptionalInt.of(latestFrameData.getFrameCount());
    }
    return OptionalInt.empty();
  }

  /**
   * Gets the number of tracking lost events since the Quest connected to the robot.
   *
   * @return The tracking lost counter value
   */
  public OptionalInt getTrackingLostCounter() {
    Data.ProtobufQuestNavDeviceData latestDeviceData = deviceDataSubscriber.get();
    if (latestDeviceData != null) {
      return OptionalInt.of(latestDeviceData.getTrackingLostCounter());
    }
    return OptionalInt.empty();
  }

  /**
   * Determines if the Quest headset is currently connected to the robot. Connection is determined
   * by how stale the last received frame from the Quest is.
   *
   * @return Boolean indicating if the Quest is connected (true) or not (false)
   */
  public boolean isConnected() {
    return Seconds.of(Timer.getTimestamp())
        .minus(Microseconds.of(frameDataSubscriber.getLastChange()))
        .lt(Milliseconds.of(50));
  }

  /**
   * Gets the latency of the Quest > Robot Connection. Returns the latency between the current time
   * and the last frame data update.
   *
   * @return The latency in milliseconds
   */
  public double getLatency() {
    return Seconds.of(Timer.getTimestamp())
        .minus(Microseconds.of(frameDataSubscriber.getLastChange()))
        .in(Milliseconds);
  }

  /*
   * Returns the Quest app's uptime timestamp. For integration with a pose estimator, use {@link
   * #getDataTimestamp()} instead!
   *
   * @return The timestamp as a double value
   */
  public OptionalDouble getAppTimestamp() {
    Data.ProtobufQuestNavFrameData latestFrameData = frameDataSubscriber.get();
    if (latestFrameData != null) {
      return OptionalDouble.of(latestFrameData.getTimestamp());
    }
    return OptionalDouble.empty();
  }

  /**
   * Gets a list of results sent by the Quest since the last call to getAllUnreadResults().
   *
   * @return returns a list of frames with pose data
   */
  public PoseFrame[] getAllUnreadPoseFrames() {
    var frameDataArray = frameDataSubscriber.readQueue();
    var result = new PoseFrame[frameDataArray.length];
    for (int i = 0; i < result.length; i++) {
      var frameData = frameDataArray[i];
      result[i] =
          new PoseFrame(
              pose2dProto.unpack(frameData.value.getPose2D()),
              Microseconds.of(frameData.serverTime).in(Seconds),
              frameData.value.getTimestamp(),
              frameData.value.getFrameCount());
    }
    return result;
  }

  /** Cleans up QuestNav responses after processing on the headset. */
  public void commandPeriodic() {
    Commands.ProtobufQuestNavCommandResponse latestCommandResponse = responseSubscriber.get();

    // if we don't have data or for some reason the response we got isn't for the command we sent,
    // skip for this loop
    if (latestCommandResponse == null || latestCommandResponse.getCommandId() != lastSentRequestId)
      return;

    if (lastProcessedResponseId != latestCommandResponse.getCommandId()) {

      if (!latestCommandResponse.getSuccess()) {
        DriverStation.reportError(
            "QuestNav command failed!\n" + latestCommandResponse.getErrorMessage(), false);
      }
      // don't double process
      lastProcessedResponseId = latestCommandResponse.getCommandId();
    }
  }

  // Get the yaw Euler angle of the headset
  private Rotation2d getUncorrectedYaw() {
    float[] eulerAngles = questEulerAngles.get();
    return Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360.0)).minus(resetHeading);
  }

  private Translation2d getUncorrectedTranslation() {
    float[] position = questPosition.get();
    return new Translation2d(position[2], -position[0]);
  }

  public Pose2d getUncorrectedPose() {
    return new Pose2d(getUncorrectedTranslation().rotateBy(resetHeading), getUncorrectedYaw());
  }

  public Pose2d getUncorrectedResetPose() {
    return this.uncorrectedResetPose;
  }
}