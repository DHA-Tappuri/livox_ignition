// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: CustomMsg.proto

#include "CustomMsg.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_CustomMsg_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_CustomPoint_CustomMsg_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_ignition_2fmsgs_2ftime_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Time_ignition_2fmsgs_2ftime_2eproto;
namespace LivoxCustomMsg {
class CustomPointDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<CustomPoint> _instance;
} _CustomPoint_default_instance_;
class CustomMsgDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<CustomMsg> _instance;
} _CustomMsg_default_instance_;
}  // namespace LivoxCustomMsg
static void InitDefaultsscc_info_CustomMsg_CustomMsg_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::LivoxCustomMsg::_CustomMsg_default_instance_;
    new (ptr) ::LivoxCustomMsg::CustomMsg();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::LivoxCustomMsg::CustomMsg::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_CustomMsg_CustomMsg_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_CustomMsg_CustomMsg_2eproto}, {
      &scc_info_Time_ignition_2fmsgs_2ftime_2eproto.base,
      &scc_info_CustomPoint_CustomMsg_2eproto.base,}};

static void InitDefaultsscc_info_CustomPoint_CustomMsg_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::LivoxCustomMsg::_CustomPoint_default_instance_;
    new (ptr) ::LivoxCustomMsg::CustomPoint();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::LivoxCustomMsg::CustomPoint::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_CustomPoint_CustomMsg_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_CustomPoint_CustomMsg_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_CustomMsg_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_CustomMsg_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_CustomMsg_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_CustomMsg_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, offset_time_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, x_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, y_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, z_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, reflectivity_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, tag_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomPoint, line_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, stamp_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, frame_id_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, time_base_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, point_num_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, lidar_id_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, rsvd_0_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, rsvd_1_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, rsvd_2_),
  PROTOBUF_FIELD_OFFSET(::LivoxCustomMsg::CustomMsg, points_),
  1,
  0,
  ~0u,
  ~0u,
  ~0u,
  ~0u,
  ~0u,
  ~0u,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::LivoxCustomMsg::CustomPoint)},
  { 12, 26, sizeof(::LivoxCustomMsg::CustomMsg)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::LivoxCustomMsg::_CustomPoint_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::LivoxCustomMsg::_CustomMsg_default_instance_),
};

const char descriptor_table_protodef_CustomMsg_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\017CustomMsg.proto\022\016LivoxCustomMsg\032\030ignit"
  "ion/msgs/time.proto\"t\n\013CustomPoint\022\023\n\013of"
  "fset_time\030\001 \001(\r\022\t\n\001x\030\002 \001(\002\022\t\n\001y\030\003 \001(\002\022\t\n"
  "\001z\030\004 \001(\002\022\024\n\014reflectivity\030\005 \001(\r\022\013\n\003tag\030\006 "
  "\001(\r\022\014\n\004line\030\007 \001(\r\"\367\001\n\tCustomMsg\022\'\n\005stamp"
  "\030\001 \001(\0132\023.ignition.msgs.TimeH\000\210\001\001\022\025\n\010fram"
  "e_id\030\002 \001(\tH\001\210\001\001\022\021\n\ttime_base\030\003 \001(\004\022\021\n\tpo"
  "int_num\030\004 \001(\r\022\020\n\010lidar_id\030\005 \001(\r\022\016\n\006rsvd_"
  "0\030\006 \001(\r\022\016\n\006rsvd_1\030\007 \001(\r\022\016\n\006rsvd_2\030\010 \001(\r\022"
  "+\n\006points\030\t \003(\0132\033.LivoxCustomMsg.CustomP"
  "ointB\010\n\006_stampB\013\n\t_frame_idb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_CustomMsg_2eproto_deps[1] = {
  &::descriptor_table_ignition_2fmsgs_2ftime_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_CustomMsg_2eproto_sccs[2] = {
  &scc_info_CustomMsg_CustomMsg_2eproto.base,
  &scc_info_CustomPoint_CustomMsg_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_CustomMsg_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_CustomMsg_2eproto = {
  false, false, descriptor_table_protodef_CustomMsg_2eproto, "CustomMsg.proto", 435,
  &descriptor_table_CustomMsg_2eproto_once, descriptor_table_CustomMsg_2eproto_sccs, descriptor_table_CustomMsg_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_CustomMsg_2eproto::offsets,
  file_level_metadata_CustomMsg_2eproto, 2, file_level_enum_descriptors_CustomMsg_2eproto, file_level_service_descriptors_CustomMsg_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_CustomMsg_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_CustomMsg_2eproto)), true);
namespace LivoxCustomMsg {

// ===================================================================

void CustomPoint::InitAsDefaultInstance() {
}
class CustomPoint::_Internal {
 public:
};

CustomPoint::CustomPoint(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:LivoxCustomMsg.CustomPoint)
}
CustomPoint::CustomPoint(const CustomPoint& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&offset_time_, &from.offset_time_,
    static_cast<size_t>(reinterpret_cast<char*>(&line_) -
    reinterpret_cast<char*>(&offset_time_)) + sizeof(line_));
  // @@protoc_insertion_point(copy_constructor:LivoxCustomMsg.CustomPoint)
}

void CustomPoint::SharedCtor() {
  ::memset(&offset_time_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&line_) -
      reinterpret_cast<char*>(&offset_time_)) + sizeof(line_));
}

CustomPoint::~CustomPoint() {
  // @@protoc_insertion_point(destructor:LivoxCustomMsg.CustomPoint)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void CustomPoint::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void CustomPoint::ArenaDtor(void* object) {
  CustomPoint* _this = reinterpret_cast< CustomPoint* >(object);
  (void)_this;
}
void CustomPoint::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void CustomPoint::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CustomPoint& CustomPoint::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_CustomPoint_CustomMsg_2eproto.base);
  return *internal_default_instance();
}


void CustomPoint::Clear() {
// @@protoc_insertion_point(message_clear_start:LivoxCustomMsg.CustomPoint)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&offset_time_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&line_) -
      reinterpret_cast<char*>(&offset_time_)) + sizeof(line_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CustomPoint::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // uint32 offset_time = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          offset_time_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // float x = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 21)) {
          x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // float y = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // float z = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 37)) {
          z_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // uint32 reflectivity = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          reflectivity_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint32 tag = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          tag_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint32 line = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          line_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* CustomPoint::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:LivoxCustomMsg.CustomPoint)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // uint32 offset_time = 1;
  if (this->offset_time() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1, this->_internal_offset_time(), target);
  }

  // float x = 2;
  if (!(this->x() <= 0 && this->x() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_x(), target);
  }

  // float y = 3;
  if (!(this->y() <= 0 && this->y() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_y(), target);
  }

  // float z = 4;
  if (!(this->z() <= 0 && this->z() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(4, this->_internal_z(), target);
  }

  // uint32 reflectivity = 5;
  if (this->reflectivity() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(5, this->_internal_reflectivity(), target);
  }

  // uint32 tag = 6;
  if (this->tag() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(6, this->_internal_tag(), target);
  }

  // uint32 line = 7;
  if (this->line() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(7, this->_internal_line(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:LivoxCustomMsg.CustomPoint)
  return target;
}

size_t CustomPoint::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:LivoxCustomMsg.CustomPoint)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // uint32 offset_time = 1;
  if (this->offset_time() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_offset_time());
  }

  // float x = 2;
  if (!(this->x() <= 0 && this->x() >= 0)) {
    total_size += 1 + 4;
  }

  // float y = 3;
  if (!(this->y() <= 0 && this->y() >= 0)) {
    total_size += 1 + 4;
  }

  // float z = 4;
  if (!(this->z() <= 0 && this->z() >= 0)) {
    total_size += 1 + 4;
  }

  // uint32 reflectivity = 5;
  if (this->reflectivity() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_reflectivity());
  }

  // uint32 tag = 6;
  if (this->tag() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_tag());
  }

  // uint32 line = 7;
  if (this->line() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_line());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void CustomPoint::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:LivoxCustomMsg.CustomPoint)
  GOOGLE_DCHECK_NE(&from, this);
  const CustomPoint* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<CustomPoint>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:LivoxCustomMsg.CustomPoint)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:LivoxCustomMsg.CustomPoint)
    MergeFrom(*source);
  }
}

void CustomPoint::MergeFrom(const CustomPoint& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:LivoxCustomMsg.CustomPoint)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.offset_time() != 0) {
    _internal_set_offset_time(from._internal_offset_time());
  }
  if (!(from.x() <= 0 && from.x() >= 0)) {
    _internal_set_x(from._internal_x());
  }
  if (!(from.y() <= 0 && from.y() >= 0)) {
    _internal_set_y(from._internal_y());
  }
  if (!(from.z() <= 0 && from.z() >= 0)) {
    _internal_set_z(from._internal_z());
  }
  if (from.reflectivity() != 0) {
    _internal_set_reflectivity(from._internal_reflectivity());
  }
  if (from.tag() != 0) {
    _internal_set_tag(from._internal_tag());
  }
  if (from.line() != 0) {
    _internal_set_line(from._internal_line());
  }
}

void CustomPoint::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:LivoxCustomMsg.CustomPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CustomPoint::CopyFrom(const CustomPoint& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:LivoxCustomMsg.CustomPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CustomPoint::IsInitialized() const {
  return true;
}

void CustomPoint::InternalSwap(CustomPoint* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CustomPoint, line_)
      + sizeof(CustomPoint::line_)
      - PROTOBUF_FIELD_OFFSET(CustomPoint, offset_time_)>(
          reinterpret_cast<char*>(&offset_time_),
          reinterpret_cast<char*>(&other->offset_time_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CustomPoint::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void CustomMsg::InitAsDefaultInstance() {
  ::LivoxCustomMsg::_CustomMsg_default_instance_._instance.get_mutable()->stamp_ = const_cast< ::ignition::msgs::Time*>(
      ::ignition::msgs::Time::internal_default_instance());
}
class CustomMsg::_Internal {
 public:
  using HasBits = decltype(std::declval<CustomMsg>()._has_bits_);
  static const ::ignition::msgs::Time& stamp(const CustomMsg* msg);
  static void set_has_stamp(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_frame_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::ignition::msgs::Time&
CustomMsg::_Internal::stamp(const CustomMsg* msg) {
  return *msg->stamp_;
}
void CustomMsg::clear_stamp() {
  if (GetArena() == nullptr && stamp_ != nullptr) {
    delete stamp_;
  }
  stamp_ = nullptr;
  _has_bits_[0] &= ~0x00000002u;
}
CustomMsg::CustomMsg(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  points_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:LivoxCustomMsg.CustomMsg)
}
CustomMsg::CustomMsg(const CustomMsg& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      points_(from.points_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_frame_id()) {
    frame_id_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from._internal_frame_id(),
      GetArena());
  }
  if (from._internal_has_stamp()) {
    stamp_ = new ::ignition::msgs::Time(*from.stamp_);
  } else {
    stamp_ = nullptr;
  }
  ::memcpy(&time_base_, &from.time_base_,
    static_cast<size_t>(reinterpret_cast<char*>(&rsvd_2_) -
    reinterpret_cast<char*>(&time_base_)) + sizeof(rsvd_2_));
  // @@protoc_insertion_point(copy_constructor:LivoxCustomMsg.CustomMsg)
}

void CustomMsg::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_CustomMsg_CustomMsg_2eproto.base);
  frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  ::memset(&stamp_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rsvd_2_) -
      reinterpret_cast<char*>(&stamp_)) + sizeof(rsvd_2_));
}

CustomMsg::~CustomMsg() {
  // @@protoc_insertion_point(destructor:LivoxCustomMsg.CustomMsg)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void CustomMsg::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  frame_id_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete stamp_;
}

void CustomMsg::ArenaDtor(void* object) {
  CustomMsg* _this = reinterpret_cast< CustomMsg* >(object);
  (void)_this;
}
void CustomMsg::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void CustomMsg::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CustomMsg& CustomMsg::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_CustomMsg_CustomMsg_2eproto.base);
  return *internal_default_instance();
}


void CustomMsg::Clear() {
// @@protoc_insertion_point(message_clear_start:LivoxCustomMsg.CustomMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  points_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      frame_id_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000002u) {
      if (GetArena() == nullptr && stamp_ != nullptr) {
        delete stamp_;
      }
      stamp_ = nullptr;
    }
  }
  ::memset(&time_base_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rsvd_2_) -
      reinterpret_cast<char*>(&time_base_)) + sizeof(rsvd_2_));
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CustomMsg::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .ignition.msgs.Time stamp = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_stamp(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // string frame_id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_frame_id();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "LivoxCustomMsg.CustomMsg.frame_id"));
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint64 time_base = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          time_base_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint32 point_num = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          point_num_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint32 lidar_id = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          lidar_id_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint32 rsvd_0 = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          rsvd_0_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint32 rsvd_1 = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          rsvd_1_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // uint32 rsvd_2 = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          rsvd_2_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .LivoxCustomMsg.CustomPoint points = 9;
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_points(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<74>(ptr));
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* CustomMsg::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:LivoxCustomMsg.CustomMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .ignition.msgs.Time stamp = 1;
  if (_internal_has_stamp()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::stamp(this), target, stream);
  }

  // string frame_id = 2;
  if (_internal_has_frame_id()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_frame_id().data(), static_cast<int>(this->_internal_frame_id().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "LivoxCustomMsg.CustomMsg.frame_id");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_frame_id(), target);
  }

  // uint64 time_base = 3;
  if (this->time_base() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt64ToArray(3, this->_internal_time_base(), target);
  }

  // uint32 point_num = 4;
  if (this->point_num() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(4, this->_internal_point_num(), target);
  }

  // uint32 lidar_id = 5;
  if (this->lidar_id() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(5, this->_internal_lidar_id(), target);
  }

  // uint32 rsvd_0 = 6;
  if (this->rsvd_0() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(6, this->_internal_rsvd_0(), target);
  }

  // uint32 rsvd_1 = 7;
  if (this->rsvd_1() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(7, this->_internal_rsvd_1(), target);
  }

  // uint32 rsvd_2 = 8;
  if (this->rsvd_2() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(8, this->_internal_rsvd_2(), target);
  }

  // repeated .LivoxCustomMsg.CustomPoint points = 9;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_points_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(9, this->_internal_points(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:LivoxCustomMsg.CustomMsg)
  return target;
}

size_t CustomMsg::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:LivoxCustomMsg.CustomMsg)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .LivoxCustomMsg.CustomPoint points = 9;
  total_size += 1UL * this->_internal_points_size();
  for (const auto& msg : this->points_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // string frame_id = 2;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_frame_id());
    }

    // .ignition.msgs.Time stamp = 1;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *stamp_);
    }

  }
  // uint64 time_base = 3;
  if (this->time_base() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt64Size(
        this->_internal_time_base());
  }

  // uint32 point_num = 4;
  if (this->point_num() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_point_num());
  }

  // uint32 lidar_id = 5;
  if (this->lidar_id() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_lidar_id());
  }

  // uint32 rsvd_0 = 6;
  if (this->rsvd_0() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_rsvd_0());
  }

  // uint32 rsvd_1 = 7;
  if (this->rsvd_1() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_rsvd_1());
  }

  // uint32 rsvd_2 = 8;
  if (this->rsvd_2() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_rsvd_2());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void CustomMsg::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:LivoxCustomMsg.CustomMsg)
  GOOGLE_DCHECK_NE(&from, this);
  const CustomMsg* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<CustomMsg>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:LivoxCustomMsg.CustomMsg)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:LivoxCustomMsg.CustomMsg)
    MergeFrom(*source);
  }
}

void CustomMsg::MergeFrom(const CustomMsg& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:LivoxCustomMsg.CustomMsg)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  points_.MergeFrom(from.points_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_frame_id(from._internal_frame_id());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_stamp()->::ignition::msgs::Time::MergeFrom(from._internal_stamp());
    }
  }
  if (from.time_base() != 0) {
    _internal_set_time_base(from._internal_time_base());
  }
  if (from.point_num() != 0) {
    _internal_set_point_num(from._internal_point_num());
  }
  if (from.lidar_id() != 0) {
    _internal_set_lidar_id(from._internal_lidar_id());
  }
  if (from.rsvd_0() != 0) {
    _internal_set_rsvd_0(from._internal_rsvd_0());
  }
  if (from.rsvd_1() != 0) {
    _internal_set_rsvd_1(from._internal_rsvd_1());
  }
  if (from.rsvd_2() != 0) {
    _internal_set_rsvd_2(from._internal_rsvd_2());
  }
}

void CustomMsg::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:LivoxCustomMsg.CustomMsg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CustomMsg::CopyFrom(const CustomMsg& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:LivoxCustomMsg.CustomMsg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CustomMsg::IsInitialized() const {
  return true;
}

void CustomMsg::InternalSwap(CustomMsg* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  points_.InternalSwap(&other->points_);
  frame_id_.Swap(&other->frame_id_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CustomMsg, rsvd_2_)
      + sizeof(CustomMsg::rsvd_2_)
      - PROTOBUF_FIELD_OFFSET(CustomMsg, stamp_)>(
          reinterpret_cast<char*>(&stamp_),
          reinterpret_cast<char*>(&other->stamp_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CustomMsg::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace LivoxCustomMsg
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::LivoxCustomMsg::CustomPoint* Arena::CreateMaybeMessage< ::LivoxCustomMsg::CustomPoint >(Arena* arena) {
  return Arena::CreateMessageInternal< ::LivoxCustomMsg::CustomPoint >(arena);
}
template<> PROTOBUF_NOINLINE ::LivoxCustomMsg::CustomMsg* Arena::CreateMaybeMessage< ::LivoxCustomMsg::CustomMsg >(Arena* arena) {
  return Arena::CreateMessageInternal< ::LivoxCustomMsg::CustomMsg >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
