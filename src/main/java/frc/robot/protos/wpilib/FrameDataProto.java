package frc.robot.protos.wpilib;

import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.generated.Data;
import us.hebi.quickbuf.Descriptors;

/** WPILib Protobuf layer for FrameData Protobuf */
public class FrameDataProto
    implements Protobuf<Data.ProtobufQuestNavFrameData, Data.ProtobufQuestNavFrameData> {
  @Override
  public Class<Data.ProtobufQuestNavFrameData> getTypeClass() {
    return Data.ProtobufQuestNavFrameData.class;
  }

  @Override
  public Descriptors.Descriptor getDescriptor() {
    return Data.ProtobufQuestNavFrameData.getDescriptor();
  }

  @Override
  public frc.robot.generated.Data.ProtobufQuestNavFrameData createMessage() {
    return Data.ProtobufQuestNavFrameData.newInstance();
  }

  @Override
  public frc.robot.generated.Data.ProtobufQuestNavFrameData unpack(Data.ProtobufQuestNavFrameData msg) {
    return msg.clone();
  }

  @Override
  public void pack(Data.ProtobufQuestNavFrameData msg, Data.ProtobufQuestNavFrameData value) {
    msg.copyFrom(value);
  }
}