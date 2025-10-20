package frc.robot.generated;

import edu.wpi.first.math.proto.Geometry2D;
import java.io.IOException;
import us.hebi.quickbuf.Descriptors;
import us.hebi.quickbuf.FieldName;
import us.hebi.quickbuf.InvalidProtocolBufferException;
import us.hebi.quickbuf.JsonSink;
import us.hebi.quickbuf.JsonSource;
import us.hebi.quickbuf.MessageFactory;
import us.hebi.quickbuf.ProtoEnum;
import us.hebi.quickbuf.ProtoMessage;
import us.hebi.quickbuf.ProtoSink;
import us.hebi.quickbuf.ProtoSource;
import us.hebi.quickbuf.ProtoUtil;
import us.hebi.quickbuf.RepeatedByte;
import us.hebi.quickbuf.Utf8String;

public final class Commands {
    private static final RepeatedByte descriptorData = ProtoUtil.decodeBase64(2020,
        "Cg5jb21tYW5kcy5wcm90bxIYcXVlc3RuYXYucHJvdG9zLmNvbW1hbmRzGhBnZW9tZXRyeTJkLnByb3Rv" + 
        "Il4KIFByb3RvYnVmUXVlc3ROYXZQb3NlUmVzZXRQYXlsb2FkEjoKC3RhcmdldF9wb3NlGAEgASgLMhku" + 
        "d3BpLnByb3RvLlByb3RvYnVmUG9zZTJkUgp0YXJnZXRQb3NlIvIBChdQcm90b2J1ZlF1ZXN0TmF2Q29t" + 
        "bWFuZBJBCgR0eXBlGAEgASgOMi0ucXVlc3RuYXYucHJvdG9zLmNvbW1hbmRzLlF1ZXN0TmF2Q29tbWFu" + 
        "ZFR5cGVSBHR5cGUSHQoKY29tbWFuZF9pZBgCIAEoDVIJY29tbWFuZElkEmoKEnBvc2VfcmVzZXRfcGF5" + 
        "bG9hZBgKIAEoCzI6LnF1ZXN0bmF2LnByb3Rvcy5jb21tYW5kcy5Qcm90b2J1ZlF1ZXN0TmF2UG9zZVJl" + 
        "c2V0UGF5bG9hZEgAUhBwb3NlUmVzZXRQYXlsb2FkQgkKB3BheWxvYWQifwofUHJvdG9idWZRdWVzdE5h" + 
        "dkNvbW1hbmRSZXNwb25zZRIdCgpjb21tYW5kX2lkGAEgASgNUgljb21tYW5kSWQSGAoHc3VjY2VzcxgC" + 
        "IAEoCFIHc3VjY2VzcxIjCg1lcnJvcl9tZXNzYWdlGAMgASgJUgxlcnJvck1lc3NhZ2UqQwoTUXVlc3RO" + 
        "YXZDb21tYW5kVHlwZRIcChhDT01NQU5EX1RZUEVfVU5TUEVDSUZJRUQQABIOCgpQT1NFX1JFU0VUEAFC" + 
        "QwolZ2cucXVlc3RuYXYucXVlc3RuYXYucHJvdG9zLmdlbmVyYXRlZKoCGVF1ZXN0TmF2LlByb3Rvcy5H" + 
        "ZW5lcmF0ZWRKvQoKBhIEAAMwAQoICgEMEgMAAxUKCAoBAhIDAgAhCggKAQgSAwMANgoJCgIIJRIDAwA2" + 
        "CggKAQgSAwQAPgoJCgIIARIDBAA+CiUKAgMAEgMHABoaGiBJbXBvcnQgZ2VvbWV0cnkgbWVzc2FnZXMK" + 
        "CkUKAgUAEgQKAA4BGjkgRW51bSBmb3IgY29tbWFuZCB0eXBlcyAoZXh0ZW5zaWJsZSBmb3IgZnV0dXJl" + 
        "IGNvbW1hbmRzKQoKCgoDBQABEgMKBRgKLwoEBQACABIDCwIfIiIgRGVmYXVsdCB2YWx1ZSByZXF1aXJl" + 
        "ZCBpbiBwcm90bzMKCgwKBQUAAgABEgMLAhoKDAoFBQACAAISAwsdHgouCgQFAAIBEgMMAhEiISBSZXNl" + 
        "dCByb2JvdCBwb3NlIHRvIHRhcmdldCBwb3NlCgoMCgUFAAIBARIDDAIMCgwKBQUAAgECEgMMDxAKLAoC" + 
        "BAASBBEAFAEaICBQYXlsb2FkIGZvciBwb3NlIHJlc2V0IGNvbW1hbmQKCgoKAwQAARIDEQgoCmcKBAQA" + 
        "AgASAxMCKxpaIFRhcmdldCBwb3NlIGluIGZpZWxkLXJlbGF0aXZlIFdQSUxpYiBjb29yZGluYXRlIHNw" + 
        "YWNlICh4IGZvcndhcmQsIHkgbGVmdCwgcm90YXRpb24gQ0NXKykKCgwKBQQAAgAGEgMTAhoKDAoFBAAC",
        "AAESAxMbJgoMCgUEAAIAAxIDEykqCiIKAgQBEgQXACQBGhYgTWFpbiBDb21tYW5kIG1lc3NhZ2UKCgoK" + 
        "AwQBARIDFwgfCiIKBAQBAgASAxkCHxoVIFRoZSB0eXBlIG9mIGNvbW1hbmQKCgwKBQQBAgAGEgMZAhUK" + 
        "DAoFBAECAAESAxkWGgoMCgUEAQIAAxIDGR0eCjAKBAQBAgESAxwCGBojIENvbW1hbmQgSUQgZm9yIHRy" + 
        "YWNraW5nL3Jlc3BvbnNlcwoKDAoFBAECAQUSAxwCCAoMCgUEAQIBARIDHAkTCgwKBQQBAgEDEgMcFhcK" + 
        "VQoEBAEIABIEHwIjAxpHIENvbW1hbmQtc3BlY2lmaWMgcGF5bG9hZCAob25seSBvbmUgd2lsbCBiZSBz" + 
        "ZXQgYmFzZWQgb24gY29tbWFuZCB0eXBlKQoKDAoFBAEIAAESAx8IDwphCgQEAQICEgMgBD0iVCBGdXR1" + 
        "cmUgcGF5bG9hZHMgY2FuIGJlIGFkZGVkIGhlcmU6CiAoQ29tbWFuZHMgd2l0aCBubyBwYXlsb2FkIGRv" + 
        "bid0IG5lZWQgYW4gZW50cnkpCgoMCgUEAQICBhIDIAQkCgwKBQQBAgIBEgMgJTcKDAoFBAECAgMSAyA6" + 
        "PAorCgIEAhIEJwAwARofIFJlc3BvbnNlIG1lc3NhZ2UgZm9yIGNvbW1hbmRzCgoKCgMEAgESAycIJwou" + 
        "CgQEAgIAEgMpAhgaISBNYXRjaGVzIHRoZSBvcmlnaW5hbCBjb21tYW5kIElECgoMCgUEAgIABRIDKQII" + 
        "CgwKBQQCAgABEgMpCRMKDAoFBAICAAMSAykWFwoxCgQEAgIBEgMsAhMaJCBXaGV0aGVyIHRoZSBjb21t" + 
        "YW5kIHdhcyBzdWNjZXNzZnVsCgoMCgUEAgIBBRIDLAIGCgwKBQQCAgEBEgMsBw4KDAoFBAICAQMSAywR" + 
        "EgovCgQEAgICEgMvAhsaIiBFcnJvciBtZXNzYWdlIGlmIHN1Y2Nlc3MgPSBmYWxzZQoKDAoFBAICAgUS" + 
        "Ay8CCAoMCgUEAgICARIDLwkWCgwKBQQCAgIDEgMvGRpiBnByb3RvMw==");

    static final Descriptors.FileDescriptor descriptor = Descriptors.FileDescriptor.internalBuildGeneratedFileFrom("commands.proto", "questnav.protos.commands", descriptorData, Geometry2D.getDescriptor());

    static final Descriptors.Descriptor questnav_protos_commands_ProtobufQuestNavPoseResetPayload_descriptor = descriptor.internalContainedType(62, 94, "ProtobufQuestNavPoseResetPayload", "questnav.protos.commands.ProtobufQuestNavPoseResetPayload");

    static final Descriptors.Descriptor questnav_protos_commands_ProtobufQuestNavCommand_descriptor = descriptor.internalContainedType(159, 242, "ProtobufQuestNavCommand", "questnav.protos.commands.ProtobufQuestNavCommand");

    static final Descriptors.Descriptor questnav_protos_commands_ProtobufQuestNavCommandResponse_descriptor = descriptor.internalContainedType(403, 127, "ProtobufQuestNavCommandResponse", "questnav.protos.commands.ProtobufQuestNavCommandResponse");

    /**
     * @return this proto file's descriptor.
     */
    public static Descriptors.FileDescriptor getDescriptor() {
        return descriptor;
    }

    /**
     * <pre>
     *  Enum for command types (extensible for future commands)
     * </pre>
     *
     * Protobuf enum {@code QuestNavCommandType}
     */
    public enum QuestNavCommandType implements ProtoEnum<QuestNavCommandType> {
        /**
         * <pre>
         *  Default value required in proto3
         * </pre>
         *
         * <code>COMMAND_TYPE_UNSPECIFIED = 0;</code>
         */
        COMMAND_TYPE_UNSPECIFIED("COMMAND_TYPE_UNSPECIFIED", 0),

        /**
         * <pre>
         *  Reset robot pose to target pose
         * </pre>
         *
         * <code>POSE_RESET = 1;</code>
         */
        POSE_RESET("POSE_RESET", 1);

        /**
         * <pre>
         *  Default value required in proto3
         * </pre>
         *
         * <code>COMMAND_TYPE_UNSPECIFIED = 0;</code>
         */
        public static final int COMMAND_TYPE_UNSPECIFIED_VALUE = 0;

        /**
         * <pre>
         *  Reset robot pose to target pose
         * </pre>
         *
         * <code>POSE_RESET = 1;</code>
         */
        public static final int POSE_RESET_VALUE = 1;

        private final String name;

        private final int number;

        private QuestNavCommandType(String name, int number) {
            this.name = name;
            this.number = number;
        }

        /**
         * @return the string representation of enum entry
         */
        @Override
        public String getName() {
            return name;
        }

        /**
         * @return the numeric wire value of this enum entry
         */
        @Override
        public int getNumber() {
            return number;
        }

        /**
         * @return a converter that maps between this enum's numeric and text representations
         */
        public static ProtoEnum.EnumConverter<QuestNavCommandType> converter() {
            return QuestNavCommandTypeConverter.INSTANCE;
        }

        /**
         * @param value The numeric wire value of the corresponding enum entry.
         * @return The enum associated with the given numeric wire value, or null if unknown.
         */
        public static QuestNavCommandType forNumber(int value) {
            return QuestNavCommandTypeConverter.INSTANCE.forNumber(value);
        }

        /**
         * @param value The numeric wire value of the corresponding enum entry.
         * @param other Fallback value in case the value is not known.
         * @return The enum associated with the given numeric wire value, or the fallback value if unknown.
         */
        public static QuestNavCommandType forNumberOr(int number, QuestNavCommandType other) {
            QuestNavCommandType value = forNumber(number);
            return value == null ? other : value;
        }

        enum QuestNavCommandTypeConverter implements ProtoEnum.EnumConverter<QuestNavCommandType> {
            INSTANCE;

            private static final QuestNavCommandType[] lookup = new QuestNavCommandType[2];

            static {
                lookup[0] = COMMAND_TYPE_UNSPECIFIED;
                lookup[1] = POSE_RESET;
            }

            @Override
            public final QuestNavCommandType forNumber(final int value) {
                if (value >= 0 && value < lookup.length) {
                    return lookup[value];
                }
                return null;
            }

            @Override
            public final QuestNavCommandType forName(final CharSequence value) {
                if (value.length() == 10) {
                    if (ProtoUtil.isEqual("POSE_RESET", value)) {
                        return POSE_RESET;
                    }
                }
                if (value.length() == 24) {
                    if (ProtoUtil.isEqual("COMMAND_TYPE_UNSPECIFIED", value)) {
                        return COMMAND_TYPE_UNSPECIFIED;
                    }
                }
                return null;
            }
        }
    }

    /**
     * <pre>
     *  Payload for pose reset command
     * </pre>
     *
     * Protobuf type {@code ProtobufQuestNavPoseResetPayload}
     */
    public static final class ProtobufQuestNavPoseResetPayload extends ProtoMessage<ProtobufQuestNavPoseResetPayload> implements Cloneable {
        private static final long serialVersionUID = 0L;

        /**
         * <pre>
         *  Target pose in field-relative WPILib coordinate space (x forward, y left, rotation CCW+)
         * </pre>
         *
         * <code>optional .wpi.proto.ProtobufPose2d target_pose = 1;</code>
         */
        private final Geometry2D.ProtobufPose2d targetPose = Geometry2D.ProtobufPose2d.newInstance();

        private ProtobufQuestNavPoseResetPayload() {
        }

        /**
         * <pre>
         *  Payload for pose reset command
         * </pre>
         *
         * @return a new empty instance of {@code ProtobufQuestNavPoseResetPayload}
         */
        public static ProtobufQuestNavPoseResetPayload newInstance() {
            return new ProtobufQuestNavPoseResetPayload();
        }

        /**
         * <pre>
         *  Target pose in field-relative WPILib coordinate space (x forward, y left, rotation CCW+)
         * </pre>
         *
         * <code>optional .wpi.proto.ProtobufPose2d target_pose = 1;</code>
         * @return whether the targetPose field is set
         */
        public boolean hasTargetPose() {
            return (bitField0_ & 0x00000001) != 0;
        }

        /**
         * <pre>
         *  Target pose in field-relative WPILib coordinate space (x forward, y left, rotation CCW+)
         * </pre>
         *
         * <code>optional .wpi.proto.ProtobufPose2d target_pose = 1;</code>
         * @return this
         */
        public ProtobufQuestNavPoseResetPayload clearTargetPose() {
            bitField0_ &= ~0x00000001;
            targetPose.clear();
            return this;
        }

        /**
         * <pre>
         *  Target pose in field-relative WPILib coordinate space (x forward, y left, rotation CCW+)
         * </pre>
         *
         * <code>optional .wpi.proto.ProtobufPose2d target_pose = 1;</code>
         *
         * This method returns the internal storage object without modifying any has state.
         * The returned object should not be modified and be treated as read-only.
         *
         * Use {@link #getMutableTargetPose()} if you want to modify it.
         *
         * @return internal storage object for reading
         */
        public Geometry2D.ProtobufPose2d getTargetPose() {
            return targetPose;
        }

        /**
         * <pre>
         *  Target pose in field-relative WPILib coordinate space (x forward, y left, rotation CCW+)
         * </pre>
         *
         * <code>optional .wpi.proto.ProtobufPose2d target_pose = 1;</code>
         *
         * This method returns the internal storage object and sets the corresponding
         * has state. The returned object will become part of this message and its
         * contents may be modified as long as the has state is not cleared.
         *
         * @return internal storage object for modifications
         */
        public Geometry2D.ProtobufPose2d getMutableTargetPose() {
            bitField0_ |= 0x00000001;
            return targetPose;
        }

        /**
         * <pre>
         *  Target pose in field-relative WPILib coordinate space (x forward, y left, rotation CCW+)
         * </pre>
         *
         * <code>optional .wpi.proto.ProtobufPose2d target_pose = 1;</code>
         * @param value the targetPose to set
         * @return this
         */
        public ProtobufQuestNavPoseResetPayload setTargetPose(
                final Geometry2D.ProtobufPose2d value) {
            bitField0_ |= 0x00000001;
            targetPose.copyFrom(value);
            return this;
        }

        @Override
        public ProtobufQuestNavPoseResetPayload copyFrom(
                final ProtobufQuestNavPoseResetPayload other) {
            cachedSize = other.cachedSize;
            if ((bitField0_ | other.bitField0_) != 0) {
                bitField0_ = other.bitField0_;
                targetPose.copyFrom(other.targetPose);
            }
            return this;
        }

        @Override
        public ProtobufQuestNavPoseResetPayload mergeFrom(
                final ProtobufQuestNavPoseResetPayload other) {
            if (other.isEmpty()) {
                return this;
            }
            cachedSize = -1;
            if (other.hasTargetPose()) {
                getMutableTargetPose().mergeFrom(other.targetPose);
            }
            return this;
        }

        @Override
        public ProtobufQuestNavPoseResetPayload clear() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            targetPose.clear();
            return this;
        }

        @Override
        public ProtobufQuestNavPoseResetPayload clearQuick() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            targetPose.clearQuick();
            return this;
        }

        @Override
        public boolean equals(Object o) {
            if (o == this) {
                return true;
            }
            if (!(o instanceof ProtobufQuestNavPoseResetPayload)) {
                return false;
            }
            ProtobufQuestNavPoseResetPayload other = (ProtobufQuestNavPoseResetPayload) o;
            return bitField0_ == other.bitField0_
                && (!hasTargetPose() || targetPose.equals(other.targetPose));
        }

        @Override
        public void writeTo(final ProtoSink output) throws IOException {
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeRawByte((byte) 10);
                output.writeMessageNoTag(targetPose);
            }
        }

        @Override
        protected int computeSerializedSize() {
            int size = 0;
            if ((bitField0_ & 0x00000001) != 0) {
                size += 1 + ProtoSink.computeMessageSizeNoTag(targetPose);
            }
            return size;
        }

        @Override
        @SuppressWarnings("fallthrough")
        public ProtobufQuestNavPoseResetPayload mergeFrom(final ProtoSource input) throws
                IOException {
            // Enabled Fall-Through Optimization (QuickBuffers)
            int tag = input.readTag();
            while (true) {
                switch (tag) {
                    case 10: {
                        // targetPose
                        input.readMessage(targetPose);
                        bitField0_ |= 0x00000001;
                        tag = input.readTag();
                        if (tag != 0) {
                            break;
                        }
                    }
                    case 0: {
                        return this;
                    }
                    default: {
                        if (!input.skipField(tag)) {
                            return this;
                        }
                        tag = input.readTag();
                        break;
                    }
                }
            }
        }

        @Override
        public void writeTo(final JsonSink output) throws IOException {
            output.beginObject();
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeMessage(FieldNames.targetPose, targetPose);
            }
            output.endObject();
        }

        @Override
        public ProtobufQuestNavPoseResetPayload mergeFrom(final JsonSource input) throws
                IOException {
            if (!input.beginObject()) {
                return this;
            }
            while (!input.isAtEnd()) {
                switch (input.readFieldHash()) {
                    case 486493634:
                    case -2084687233: {
                        if (input.isAtField(FieldNames.targetPose)) {
                            if (!input.trySkipNullValue()) {
                                input.readMessage(targetPose);
                                bitField0_ |= 0x00000001;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    default: {
                        input.skipUnknownField();
                        break;
                    }
                }
            }
            input.endObject();
            return this;
        }

        @Override
        public ProtobufQuestNavPoseResetPayload clone() {
            return new ProtobufQuestNavPoseResetPayload().copyFrom(this);
        }

        @Override
        public boolean isEmpty() {
            return ((bitField0_) == 0);
        }

        public static ProtobufQuestNavPoseResetPayload parseFrom(final byte[] data) throws
                InvalidProtocolBufferException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavPoseResetPayload(), data).checkInitialized();
        }

        public static ProtobufQuestNavPoseResetPayload parseFrom(final ProtoSource input) throws
                IOException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavPoseResetPayload(), input).checkInitialized();
        }

        public static ProtobufQuestNavPoseResetPayload parseFrom(final JsonSource input) throws
                IOException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavPoseResetPayload(), input).checkInitialized();
        }

        /**
         * @return factory for creating ProtobufQuestNavPoseResetPayload messages
         */
        public static MessageFactory<ProtobufQuestNavPoseResetPayload> getFactory() {
            return ProtobufQuestNavPoseResetPayloadFactory.INSTANCE;
        }

        /**
         * @return this type's descriptor.
         */
        public static Descriptors.Descriptor getDescriptor() {
            return Commands.questnav_protos_commands_ProtobufQuestNavPoseResetPayload_descriptor;
        }

        private enum ProtobufQuestNavPoseResetPayloadFactory implements MessageFactory<ProtobufQuestNavPoseResetPayload> {
            INSTANCE;

            @Override
            public ProtobufQuestNavPoseResetPayload create() {
                return ProtobufQuestNavPoseResetPayload.newInstance();
            }
        }

        /**
         * Contains name constants used for serializing JSON
         */
        static class FieldNames {
            static final FieldName targetPose = FieldName.forField("targetPose", "target_pose");
        }
    }

    /**
     * <pre>
     *  Main Command message
     * </pre>
     *
     * Protobuf type {@code ProtobufQuestNavCommand}
     */
    public static final class ProtobufQuestNavCommand extends ProtoMessage<ProtobufQuestNavCommand> implements Cloneable {
        private static final long serialVersionUID = 0L;

        /**
         * <pre>
         *  Command ID for tracking/responses
         * </pre>
         *
         * <code>optional uint32 command_id = 2;</code>
         */
        private int commandId;

        /**
         * <pre>
         *  The type of command
         * </pre>
         *
         * <code>optional .questnav.protos.commands.QuestNavCommandType type = 1;</code>
         */
        private int type;

        /**
         * <pre>
         *  Future payloads can be added here:
         *  (Commands with no payload don't need an entry)
         * </pre>
         *
         * <code>optional .questnav.protos.commands.ProtobufQuestNavPoseResetPayload pose_reset_payload = 10;</code>
         */
        private final ProtobufQuestNavPoseResetPayload poseResetPayload = ProtobufQuestNavPoseResetPayload.newInstance();

        private ProtobufQuestNavCommand() {
        }

        /**
         * <pre>
         *  Main Command message
         * </pre>
         *
         * @return a new empty instance of {@code ProtobufQuestNavCommand}
         */
        public static ProtobufQuestNavCommand newInstance() {
            return new ProtobufQuestNavCommand();
        }

        public boolean hasPayload() {
            return (((bitField0_ & 0x00000001)) != 0);
        }

        public ProtobufQuestNavCommand clearPayload() {
            if (hasPayload()) {
                clearPoseResetPayload();
            }
            return this;
        }

        /**
         * <pre>
         *  Command ID for tracking/responses
         * </pre>
         *
         * <code>optional uint32 command_id = 2;</code>
         * @return whether the commandId field is set
         */
        public boolean hasCommandId() {
            return (bitField0_ & 0x00000002) != 0;
        }

        /**
         * <pre>
         *  Command ID for tracking/responses
         * </pre>
         *
         * <code>optional uint32 command_id = 2;</code>
         * @return this
         */
        public ProtobufQuestNavCommand clearCommandId() {
            bitField0_ &= ~0x00000002;
            commandId = 0;
            return this;
        }

        /**
         * <pre>
         *  Command ID for tracking/responses
         * </pre>
         *
         * <code>optional uint32 command_id = 2;</code>
         * @return the commandId
         */
        public int getCommandId() {
            return commandId;
        }

        /**
         * <pre>
         *  Command ID for tracking/responses
         * </pre>
         *
         * <code>optional uint32 command_id = 2;</code>
         * @param value the commandId to set
         * @return this
         */
        public ProtobufQuestNavCommand setCommandId(final int value) {
            bitField0_ |= 0x00000002;
            commandId = value;
            return this;
        }

        /**
         * <pre>
         *  The type of command
         * </pre>
         *
         * <code>optional .questnav.protos.commands.QuestNavCommandType type = 1;</code>
         * @return whether the type field is set
         */
        public boolean hasType() {
            return (bitField0_ & 0x00000004) != 0;
        }

        /**
         * <pre>
         *  The type of command
         * </pre>
         *
         * <code>optional .questnav.protos.commands.QuestNavCommandType type = 1;</code>
         * @return this
         */
        public ProtobufQuestNavCommand clearType() {
            bitField0_ &= ~0x00000004;
            type = 0;
            return this;
        }

        /**
         * <pre>
         *  The type of command
         * </pre>
         *
         * <code>optional .questnav.protos.commands.QuestNavCommandType type = 1;</code>
         * @return the type
         */
        public QuestNavCommandType getType() {
            return QuestNavCommandType.forNumber(type);
        }

        /**
         * Gets the value of the internal enum store. The result is
         * equivalent to {@link ProtobufQuestNavCommand#getType()}.getNumber().
         *
         * @return numeric wire representation
         */
        public int getTypeValue() {
            return type;
        }

        /**
         * Sets the value of the internal enum store. This does not
         * do any validity checks, so be sure to use appropriate value
         * constants from {@link QuestNavCommandType}. Setting an invalid value
         * can cause {@link ProtobufQuestNavCommand#getType()} to return null
         *
         * @param value the numeric wire value to set
         * @return this
         */
        public ProtobufQuestNavCommand setTypeValue(final int value) {
            bitField0_ |= 0x00000004;
            type = value;
            return this;
        }

        /**
         * <pre>
         *  The type of command
         * </pre>
         *
         * <code>optional .questnav.protos.commands.QuestNavCommandType type = 1;</code>
         * @param value the type to set
         * @return this
         */
        public ProtobufQuestNavCommand setType(final QuestNavCommandType value) {
            bitField0_ |= 0x00000004;
            type = value.getNumber();
            return this;
        }

        /**
         * <pre>
         *  Future payloads can be added here:
         *  (Commands with no payload don't need an entry)
         * </pre>
         *
         * <code>optional .questnav.protos.commands.ProtobufQuestNavPoseResetPayload pose_reset_payload = 10;</code>
         * @return whether the poseResetPayload field is set
         */
        public boolean hasPoseResetPayload() {
            return (bitField0_ & 0x00000001) != 0;
        }

        /**
         * <pre>
         *  Future payloads can be added here:
         *  (Commands with no payload don't need an entry)
         * </pre>
         *
         * <code>optional .questnav.protos.commands.ProtobufQuestNavPoseResetPayload pose_reset_payload = 10;</code>
         * @return this
         */
        public ProtobufQuestNavCommand clearPoseResetPayload() {
            bitField0_ &= ~0x00000001;
            poseResetPayload.clear();
            return this;
        }

        /**
         * <pre>
         *  Future payloads can be added here:
         *  (Commands with no payload don't need an entry)
         * </pre>
         *
         * <code>optional .questnav.protos.commands.ProtobufQuestNavPoseResetPayload pose_reset_payload = 10;</code>
         *
         * This method returns the internal storage object without modifying any has state.
         * The returned object should not be modified and be treated as read-only.
         *
         * Use {@link #getMutablePoseResetPayload()} if you want to modify it.
         *
         * @return internal storage object for reading
         */
        public ProtobufQuestNavPoseResetPayload getPoseResetPayload() {
            return poseResetPayload;
        }

        /**
         * <pre>
         *  Future payloads can be added here:
         *  (Commands with no payload don't need an entry)
         * </pre>
         *
         * <code>optional .questnav.protos.commands.ProtobufQuestNavPoseResetPayload pose_reset_payload = 10;</code>
         *
         * This method returns the internal storage object and sets the corresponding
         * has state. The returned object will become part of this message and its
         * contents may be modified as long as the has state is not cleared.
         *
         * @return internal storage object for modifications
         */
        public ProtobufQuestNavPoseResetPayload getMutablePoseResetPayload() {
            bitField0_ |= 0x00000001;
            return poseResetPayload;
        }

        /**
         * <pre>
         *  Future payloads can be added here:
         *  (Commands with no payload don't need an entry)
         * </pre>
         *
         * <code>optional .questnav.protos.commands.ProtobufQuestNavPoseResetPayload pose_reset_payload = 10;</code>
         * @param value the poseResetPayload to set
         * @return this
         */
        public ProtobufQuestNavCommand setPoseResetPayload(
                final ProtobufQuestNavPoseResetPayload value) {
            bitField0_ |= 0x00000001;
            poseResetPayload.copyFrom(value);
            return this;
        }

        @Override
        public ProtobufQuestNavCommand copyFrom(final ProtobufQuestNavCommand other) {
            cachedSize = other.cachedSize;
            if ((bitField0_ | other.bitField0_) != 0) {
                bitField0_ = other.bitField0_;
                commandId = other.commandId;
                type = other.type;
                poseResetPayload.copyFrom(other.poseResetPayload);
            }
            return this;
        }

        @Override
        public ProtobufQuestNavCommand mergeFrom(final ProtobufQuestNavCommand other) {
            if (other.isEmpty()) {
                return this;
            }
            cachedSize = -1;
            if (other.hasCommandId()) {
                setCommandId(other.commandId);
            }
            if (other.hasType()) {
                setTypeValue(other.type);
            }
            if (other.hasPoseResetPayload()) {
                getMutablePoseResetPayload().mergeFrom(other.poseResetPayload);
            }
            return this;
        }

        @Override
        public ProtobufQuestNavCommand clear() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            commandId = 0;
            type = 0;
            poseResetPayload.clear();
            return this;
        }

        @Override
        public ProtobufQuestNavCommand clearQuick() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            poseResetPayload.clearQuick();
            return this;
        }

        @Override
        public boolean equals(Object o) {
            if (o == this) {
                return true;
            }
            if (!(o instanceof ProtobufQuestNavCommand)) {
                return false;
            }
            ProtobufQuestNavCommand other = (ProtobufQuestNavCommand) o;
            return bitField0_ == other.bitField0_
                && (!hasCommandId() || commandId == other.commandId)
                && (!hasType() || type == other.type)
                && (!hasPoseResetPayload() || poseResetPayload.equals(other.poseResetPayload));
        }

        @Override
        public void writeTo(final ProtoSink output) throws IOException {
            if ((bitField0_ & 0x00000002) != 0) {
                output.writeRawByte((byte) 16);
                output.writeUInt32NoTag(commandId);
            }
            if ((bitField0_ & 0x00000004) != 0) {
                output.writeRawByte((byte) 8);
                output.writeEnumNoTag(type);
            }
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeRawByte((byte) 82);
                output.writeMessageNoTag(poseResetPayload);
            }
        }

        @Override
        protected int computeSerializedSize() {
            int size = 0;
            if ((bitField0_ & 0x00000002) != 0) {
                size += 1 + ProtoSink.computeUInt32SizeNoTag(commandId);
            }
            if ((bitField0_ & 0x00000004) != 0) {
                size += 1 + ProtoSink.computeEnumSizeNoTag(type);
            }
            if ((bitField0_ & 0x00000001) != 0) {
                size += 1 + ProtoSink.computeMessageSizeNoTag(poseResetPayload);
            }
            return size;
        }

        @Override
        @SuppressWarnings("fallthrough")
        public ProtobufQuestNavCommand mergeFrom(final ProtoSource input) throws IOException {
            // Enabled Fall-Through Optimization (QuickBuffers)
            int tag = input.readTag();
            while (true) {
                switch (tag) {
                    case 16: {
                        // commandId
                        commandId = input.readUInt32();
                        bitField0_ |= 0x00000002;
                        tag = input.readTag();
                        if (tag != 8) {
                            break;
                        }
                    }
                    case 8: {
                        // type
                        final int value = input.readInt32();
                        if (QuestNavCommandType.forNumber(value) != null) {
                            type = value;
                            bitField0_ |= 0x00000004;
                        }
                        tag = input.readTag();
                        if (tag != 82) {
                            break;
                        }
                    }
                    case 82: {
                        // poseResetPayload
                        input.readMessage(poseResetPayload);
                        bitField0_ |= 0x00000001;
                        tag = input.readTag();
                        if (tag != 0) {
                            break;
                        }
                    }
                    case 0: {
                        return this;
                    }
                    default: {
                        if (!input.skipField(tag)) {
                            return this;
                        }
                        tag = input.readTag();
                        break;
                    }
                }
            }
        }

        @Override
        public void writeTo(final JsonSink output) throws IOException {
            output.beginObject();
            if ((bitField0_ & 0x00000002) != 0) {
                output.writeUInt32(FieldNames.commandId, commandId);
            }
            if ((bitField0_ & 0x00000004) != 0) {
                output.writeEnum(FieldNames.type, type, QuestNavCommandType.converter());
            }
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeMessage(FieldNames.poseResetPayload, poseResetPayload);
            }
            output.endObject();
        }

        @Override
        public ProtobufQuestNavCommand mergeFrom(final JsonSource input) throws IOException {
            if (!input.beginObject()) {
                return this;
            }
            while (!input.isAtEnd()) {
                switch (input.readFieldHash()) {
                    case -1498725946:
                    case 784157327: {
                        if (input.isAtField(FieldNames.commandId)) {
                            if (!input.trySkipNullValue()) {
                                commandId = input.readUInt32();
                                bitField0_ |= 0x00000002;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case 3575610: {
                        if (input.isAtField(FieldNames.type)) {
                            if (!input.trySkipNullValue()) {
                                final QuestNavCommandType value = input.readEnum(QuestNavCommandType.converter());
                                if (value != null) {
                                    type = value.getNumber();
                                    bitField0_ |= 0x00000004;
                                } else {
                                    input.skipUnknownEnumValue();
                                }
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case -305975184:
                    case -1565360272: {
                        if (input.isAtField(FieldNames.poseResetPayload)) {
                            if (!input.trySkipNullValue()) {
                                input.readMessage(poseResetPayload);
                                bitField0_ |= 0x00000001;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    default: {
                        input.skipUnknownField();
                        break;
                    }
                }
            }
            input.endObject();
            return this;
        }

        @Override
        public ProtobufQuestNavCommand clone() {
            return new ProtobufQuestNavCommand().copyFrom(this);
        }

        @Override
        public boolean isEmpty() {
            return ((bitField0_) == 0);
        }

        public static ProtobufQuestNavCommand parseFrom(final byte[] data) throws
                InvalidProtocolBufferException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavCommand(), data).checkInitialized();
        }

        public static ProtobufQuestNavCommand parseFrom(final ProtoSource input) throws
                IOException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavCommand(), input).checkInitialized();
        }

        public static ProtobufQuestNavCommand parseFrom(final JsonSource input) throws IOException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavCommand(), input).checkInitialized();
        }

        /**
         * @return factory for creating ProtobufQuestNavCommand messages
         */
        public static MessageFactory<ProtobufQuestNavCommand> getFactory() {
            return ProtobufQuestNavCommandFactory.INSTANCE;
        }

        /**
         * @return this type's descriptor.
         */
        public static Descriptors.Descriptor getDescriptor() {
            return Commands.questnav_protos_commands_ProtobufQuestNavCommand_descriptor;
        }

        private enum ProtobufQuestNavCommandFactory implements MessageFactory<ProtobufQuestNavCommand> {
            INSTANCE;

            @Override
            public ProtobufQuestNavCommand create() {
                return ProtobufQuestNavCommand.newInstance();
            }
        }

        /**
         * Contains name constants used for serializing JSON
         */
        static class FieldNames {
            static final FieldName commandId = FieldName.forField("commandId", "command_id");

            static final FieldName type = FieldName.forField("type");

            static final FieldName poseResetPayload = FieldName.forField("poseResetPayload", "pose_reset_payload");
        }
    }

    /**
     * <pre>
     *  Response message for commands
     * </pre>
     *
     * Protobuf type {@code ProtobufQuestNavCommandResponse}
     */
    public static final class ProtobufQuestNavCommandResponse extends ProtoMessage<ProtobufQuestNavCommandResponse> implements Cloneable {
        private static final long serialVersionUID = 0L;

        /**
         * <pre>
         *  Matches the original command ID
         * </pre>
         *
         * <code>optional uint32 command_id = 1;</code>
         */
        private int commandId;

        /**
         * <pre>
         *  Whether the command was successful
         * </pre>
         *
         * <code>optional bool success = 2;</code>
         */
        private boolean success;

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         */
        private final Utf8String errorMessage = Utf8String.newEmptyInstance();

        private ProtobufQuestNavCommandResponse() {
        }

        /**
         * <pre>
         *  Response message for commands
         * </pre>
         *
         * @return a new empty instance of {@code ProtobufQuestNavCommandResponse}
         */
        public static ProtobufQuestNavCommandResponse newInstance() {
            return new ProtobufQuestNavCommandResponse();
        }

        /**
         * <pre>
         *  Matches the original command ID
         * </pre>
         *
         * <code>optional uint32 command_id = 1;</code>
         * @return whether the commandId field is set
         */
        public boolean hasCommandId() {
            return (bitField0_ & 0x00000001) != 0;
        }

        /**
         * <pre>
         *  Matches the original command ID
         * </pre>
         *
         * <code>optional uint32 command_id = 1;</code>
         * @return this
         */
        public ProtobufQuestNavCommandResponse clearCommandId() {
            bitField0_ &= ~0x00000001;
            commandId = 0;
            return this;
        }

        /**
         * <pre>
         *  Matches the original command ID
         * </pre>
         *
         * <code>optional uint32 command_id = 1;</code>
         * @return the commandId
         */
        public int getCommandId() {
            return commandId;
        }

        /**
         * <pre>
         *  Matches the original command ID
         * </pre>
         *
         * <code>optional uint32 command_id = 1;</code>
         * @param value the commandId to set
         * @return this
         */
        public ProtobufQuestNavCommandResponse setCommandId(final int value) {
            bitField0_ |= 0x00000001;
            commandId = value;
            return this;
        }

        /**
         * <pre>
         *  Whether the command was successful
         * </pre>
         *
         * <code>optional bool success = 2;</code>
         * @return whether the success field is set
         */
        public boolean hasSuccess() {
            return (bitField0_ & 0x00000002) != 0;
        }

        /**
         * <pre>
         *  Whether the command was successful
         * </pre>
         *
         * <code>optional bool success = 2;</code>
         * @return this
         */
        public ProtobufQuestNavCommandResponse clearSuccess() {
            bitField0_ &= ~0x00000002;
            success = false;
            return this;
        }

        /**
         * <pre>
         *  Whether the command was successful
         * </pre>
         *
         * <code>optional bool success = 2;</code>
         * @return the success
         */
        public boolean getSuccess() {
            return success;
        }

        /**
         * <pre>
         *  Whether the command was successful
         * </pre>
         *
         * <code>optional bool success = 2;</code>
         * @param value the success to set
         * @return this
         */
        public ProtobufQuestNavCommandResponse setSuccess(final boolean value) {
            bitField0_ |= 0x00000002;
            success = value;
            return this;
        }

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         * @return whether the errorMessage field is set
         */
        public boolean hasErrorMessage() {
            return (bitField0_ & 0x00000004) != 0;
        }

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         * @return this
         */
        public ProtobufQuestNavCommandResponse clearErrorMessage() {
            bitField0_ &= ~0x00000004;
            errorMessage.clear();
            return this;
        }

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         * @return the errorMessage
         */
        public String getErrorMessage() {
            return errorMessage.getString();
        }

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         * @return internal {@code Utf8String} representation of errorMessage for reading
         */
        public Utf8String getErrorMessageBytes() {
            return this.errorMessage;
        }

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         * @return internal {@code Utf8String} representation of errorMessage for modifications
         */
        public Utf8String getMutableErrorMessageBytes() {
            bitField0_ |= 0x00000004;
            return this.errorMessage;
        }

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         * @param value the errorMessage to set
         * @return this
         */
        public ProtobufQuestNavCommandResponse setErrorMessage(final CharSequence value) {
            bitField0_ |= 0x00000004;
            errorMessage.copyFrom(value);
            return this;
        }

        /**
         * <pre>
         *  Error message if success = false
         * </pre>
         *
         * <code>optional string error_message = 3;</code>
         * @param value the errorMessage to set
         * @return this
         */
        public ProtobufQuestNavCommandResponse setErrorMessage(final Utf8String value) {
            bitField0_ |= 0x00000004;
            errorMessage.copyFrom(value);
            return this;
        }

        @Override
        public ProtobufQuestNavCommandResponse copyFrom(
                final ProtobufQuestNavCommandResponse other) {
            cachedSize = other.cachedSize;
            if ((bitField0_ | other.bitField0_) != 0) {
                bitField0_ = other.bitField0_;
                commandId = other.commandId;
                success = other.success;
                errorMessage.copyFrom(other.errorMessage);
            }
            return this;
        }

        @Override
        public ProtobufQuestNavCommandResponse mergeFrom(
                final ProtobufQuestNavCommandResponse other) {
            if (other.isEmpty()) {
                return this;
            }
            cachedSize = -1;
            if (other.hasCommandId()) {
                setCommandId(other.commandId);
            }
            if (other.hasSuccess()) {
                setSuccess(other.success);
            }
            if (other.hasErrorMessage()) {
                getMutableErrorMessageBytes().copyFrom(other.errorMessage);
            }
            return this;
        }

        @Override
        public ProtobufQuestNavCommandResponse clear() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            commandId = 0;
            success = false;
            errorMessage.clear();
            return this;
        }

        @Override
        public ProtobufQuestNavCommandResponse clearQuick() {
            if (isEmpty()) {
                return this;
            }
            cachedSize = -1;
            bitField0_ = 0;
            errorMessage.clear();
            return this;
        }

        @Override
        public boolean equals(Object o) {
            if (o == this) {
                return true;
            }
            if (!(o instanceof ProtobufQuestNavCommandResponse)) {
                return false;
            }
            ProtobufQuestNavCommandResponse other = (ProtobufQuestNavCommandResponse) o;
            return bitField0_ == other.bitField0_
                && (!hasCommandId() || commandId == other.commandId)
                && (!hasSuccess() || success == other.success)
                && (!hasErrorMessage() || errorMessage.equals(other.errorMessage));
        }

        @Override
        public void writeTo(final ProtoSink output) throws IOException {
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeRawByte((byte) 8);
                output.writeUInt32NoTag(commandId);
            }
            if ((bitField0_ & 0x00000002) != 0) {
                output.writeRawByte((byte) 16);
                output.writeBoolNoTag(success);
            }
            if ((bitField0_ & 0x00000004) != 0) {
                output.writeRawByte((byte) 26);
                output.writeStringNoTag(errorMessage);
            }
        }

        @Override
        protected int computeSerializedSize() {
            int size = 0;
            if ((bitField0_ & 0x00000001) != 0) {
                size += 1 + ProtoSink.computeUInt32SizeNoTag(commandId);
            }
            if ((bitField0_ & 0x00000002) != 0) {
                size += 2;
            }
            if ((bitField0_ & 0x00000004) != 0) {
                size += 1 + ProtoSink.computeStringSizeNoTag(errorMessage);
            }
            return size;
        }

        @Override
        @SuppressWarnings("fallthrough")
        public ProtobufQuestNavCommandResponse mergeFrom(final ProtoSource input) throws
                IOException {
            // Enabled Fall-Through Optimization (QuickBuffers)
            int tag = input.readTag();
            while (true) {
                switch (tag) {
                    case 8: {
                        // commandId
                        commandId = input.readUInt32();
                        bitField0_ |= 0x00000001;
                        tag = input.readTag();
                        if (tag != 16) {
                            break;
                        }
                    }
                    case 16: {
                        // success
                        success = input.readBool();
                        bitField0_ |= 0x00000002;
                        tag = input.readTag();
                        if (tag != 26) {
                            break;
                        }
                    }
                    case 26: {
                        // errorMessage
                        input.readString(errorMessage);
                        bitField0_ |= 0x00000004;
                        tag = input.readTag();
                        if (tag != 0) {
                            break;
                        }
                    }
                    case 0: {
                        return this;
                    }
                    default: {
                        if (!input.skipField(tag)) {
                            return this;
                        }
                        tag = input.readTag();
                        break;
                    }
                }
            }
        }

        @Override
        public void writeTo(final JsonSink output) throws IOException {
            output.beginObject();
            if ((bitField0_ & 0x00000001) != 0) {
                output.writeUInt32(FieldNames.commandId, commandId);
            }
            if ((bitField0_ & 0x00000002) != 0) {
                output.writeBool(FieldNames.success, success);
            }
            if ((bitField0_ & 0x00000004) != 0) {
                output.writeString(FieldNames.errorMessage, errorMessage);
            }
            output.endObject();
        }

        @Override
        public ProtobufQuestNavCommandResponse mergeFrom(final JsonSource input) throws
                IOException {
            if (!input.beginObject()) {
                return this;
            }
            while (!input.isAtEnd()) {
                switch (input.readFieldHash()) {
                    case -1498725946:
                    case 784157327: {
                        if (input.isAtField(FieldNames.commandId)) {
                            if (!input.trySkipNullValue()) {
                                commandId = input.readUInt32();
                                bitField0_ |= 0x00000001;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case -1867169789: {
                        if (input.isAtField(FieldNames.success)) {
                            if (!input.trySkipNullValue()) {
                                success = input.readBool();
                                bitField0_ |= 0x00000002;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    case 1203236063:
                    case -1938755376: {
                        if (input.isAtField(FieldNames.errorMessage)) {
                            if (!input.trySkipNullValue()) {
                                input.readString(errorMessage);
                                bitField0_ |= 0x00000004;
                            }
                        } else {
                            input.skipUnknownField();
                        }
                        break;
                    }
                    default: {
                        input.skipUnknownField();
                        break;
                    }
                }
            }
            input.endObject();
            return this;
        }

        @Override
        public ProtobufQuestNavCommandResponse clone() {
            return new ProtobufQuestNavCommandResponse().copyFrom(this);
        }

        @Override
        public boolean isEmpty() {
            return ((bitField0_) == 0);
        }

        public static ProtobufQuestNavCommandResponse parseFrom(final byte[] data) throws
                InvalidProtocolBufferException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavCommandResponse(), data).checkInitialized();
        }

        public static ProtobufQuestNavCommandResponse parseFrom(final ProtoSource input) throws
                IOException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavCommandResponse(), input).checkInitialized();
        }

        public static ProtobufQuestNavCommandResponse parseFrom(final JsonSource input) throws
                IOException {
            return ProtoMessage.mergeFrom(new ProtobufQuestNavCommandResponse(), input).checkInitialized();
        }

        /**
         * @return factory for creating ProtobufQuestNavCommandResponse messages
         */
        public static MessageFactory<ProtobufQuestNavCommandResponse> getFactory() {
            return ProtobufQuestNavCommandResponseFactory.INSTANCE;
        }

        /**
         * @return this type's descriptor.
         */
        public static Descriptors.Descriptor getDescriptor() {
            return Commands.questnav_protos_commands_ProtobufQuestNavCommandResponse_descriptor;
        }

        private enum ProtobufQuestNavCommandResponseFactory implements MessageFactory<ProtobufQuestNavCommandResponse> {
            INSTANCE;

            @Override
            public ProtobufQuestNavCommandResponse create() {
                return ProtobufQuestNavCommandResponse.newInstance();
            }
        }

        /**
         * Contains name constants used for serializing JSON
         */
        static class FieldNames {
            static final FieldName commandId = FieldName.forField("commandId", "command_id");

            static final FieldName success = FieldName.forField("success");

            static final FieldName errorMessage = FieldName.forField("errorMessage", "error_message");
        }
    }
}
