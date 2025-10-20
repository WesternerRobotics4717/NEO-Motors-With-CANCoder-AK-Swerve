package frc.robot.protos.wpilib;

import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.generated.Data;
import us.hebi.quickbuf.Descriptors;

/** WPILib Protobuf layer for DeviceData Protobuf */
public class DeviceDataProto
    implements Protobuf<Data.ProtobufQuestNavDeviceData, Data.ProtobufQuestNavDeviceData> {
  @Override
  public Class<Data.ProtobufQuestNavDeviceData> getTypeClass() {
    return Data.ProtobufQuestNavDeviceData.class;
  }

  @Override
  public Descriptors.Descriptor getDescriptor() {
    return Data.ProtobufQuestNavDeviceData.getDescriptor();
  }

  @Override
  public frc.robot.generated.Data.ProtobufQuestNavDeviceData createMessage() {
    return Data.ProtobufQuestNavDeviceData.newInstance();
  }

  @Override
  public frc.robot.generated.Data.ProtobufQuestNavDeviceData unpack(Data.ProtobufQuestNavDeviceData msg) {
    return msg.clone();
  }

  @Override
  public void pack(Data.ProtobufQuestNavDeviceData msg, Data.ProtobufQuestNavDeviceData value) {
    msg.copyFrom(value);
  }
}
