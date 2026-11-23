#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <utility>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <taskgraph/Error.hpp>

namespace pg {
namespace {

std::string LowerExtension(const std::string& path) {
  std::string ext = std::filesystem::path(path).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext;
}

bool IsPly(const std::string& path) { return LowerExtension(path) == ".ply"; }

struct KnownField {
  const char* name;
  PointFields bit;
};

constexpr KnownField kKnownFields[] = {
    {"x", kFieldX},
    {"y", kFieldY},
    {"z", kFieldZ},
    {"normal_x", kFieldNormalX},
    {"normal_y", kFieldNormalY},
    {"normal_z", kFieldNormalZ},
    {"intensity", kFieldIntensity},
    {"rgb", kFieldRgb},
    {"rgba", kFieldRgba},
    {"curvature", kFieldCurvature},
    {"label", kFieldLabel},
    {"h", kFieldH},
    {"s", kFieldS},
    {"v", kFieldV},
    {"l", kFieldL},
    {"a", kFieldA},
    {"b", kFieldB},
    {"range", kFieldRange},
    {"vp_x", kFieldVpX},
    {"vp_y", kFieldVpY},
    {"vp_z", kFieldVpZ},
    {"scale", kFieldScale},
    {"strength", kFieldStrength},
    {"radius", kFieldRadius},
    {"confidence", kFieldConfidence},
    {"intensity_variance", kFieldIntensityVariance},
};

bool FieldsFromCloud2(const pcl::PCLPointCloud2& blob, PointFields& fields) {
  PointFields bits = 0;
  for (const auto& field : blob.fields) {
    if (!field.name.empty() && field.name.front() == '_') {
      continue;
    }
    bool known = false;
    for (const auto& candidate : kKnownFields) {
      if (field.name == candidate.name) {
        bits |= candidate.bit;
        known = true;
        break;
      }
    }
    if (!known) {
      return false;
    }
  }
  fields = bits;
  return bits != 0;
}

bool ReadHeaderFields(const std::string& path, PointFields& fields) {
  pcl::PCLPointCloud2 blob;
  if (IsPly(path)) {
    pcl::PLYReader reader;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version = 0;
    int data_type = 0;
    unsigned int data_index = 0;
    if (reader.readHeader(path, blob, origin, orientation, version, data_type,
                          data_index) != 0) {
      return false;
    }
  } else {
    pcl::PCDReader reader;
    if (reader.readHeader(path, blob) != 0) {
      return false;
    }
  }
  return FieldsFromCloud2(blob, fields);
}

const PointTypeDesc* MatchType(PointFields fields) {
  const PointTypeDesc* exact = nullptr;
  int exact_count = 0;
  const PointTypeDesc* superset = nullptr;
  int superset_count = 0;
  for (const auto& desc : PointTypeRegistry::Instance().All()) {
    if (desc.fields == fields) {
      exact = &desc;
      ++exact_count;
    } else if ((desc.fields & fields) == fields) {
      superset = &desc;
      ++superset_count;
    }
  }
  if (exact_count == 1) {
    return exact;
  }
  if (exact_count > 1) {
    return nullptr;
  }
  return superset_count == 1 ? superset : nullptr;
}

const PointTypeDesc* InferType(const std::string& path) {
  PointFields fields = 0;
  if (!ReadHeaderFields(path, fields)) {
    return nullptr;
  }
  return MatchType(fields);
}

bool LoadCloud2(const std::string& path, pcl::PCLPointCloud2& blob) {
  if (IsPly(path)) {
    return pcl::io::loadPLYFile(path, blob) == 0;
  }
  return pcl::io::loadPCDFile(path, blob) == 0;
}

}  // namespace

PointCloudReaderNode::PointCloudReaderNode()
    : outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = AllTypes();
  _path = addProperty("path", std::string(), FileHints());
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, true));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
}

void PointCloudReaderNode::setPath(std::string path) {
  _path->value = std::move(path);
  refreshPortTypes();
}

void PointCloudReaderNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

const PointTypeDesc* PointCloudReaderNode::ResolveAuto() const {
  const std::string path = std::get<std::string>(_path->value);
  if (path.empty()) {
    return nullptr;
  }
  if (_cache_valid && _cache_path == path) {
    return _cache_desc;
  }
  _cache_path = path;
  _cache_valid = true;
  try {
    _cache_desc = InferType(path);
  } catch (...) {
    _cache_desc = nullptr;
  }
  return _cache_desc;
}

const PointTypeDesc* PointCloudReaderNode::SelectedType() const {
  const int index = std::get<int>(_point_type->value);
  if (index > 0) {
    return PointTypeAt(index, _types, true);
  }
  return ResolveAuto();
}

tg::NodeDataType PointCloudReaderNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

int PointCloudReaderNode::portTypeProperty(tg::PortType port_type,
                                           unsigned int index) const {
  return port_type == tg::PortType::Out && index == kOutIndex
             ? _point_type_index
             : -1;
}

void PointCloudReaderNode::compute() {
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr) {
    throw tg::NodeError(
        "PointCloudReader: cannot resolve point type; select a file or set "
        "point_type");
  }
  const std::string path = std::get<std::string>(_path->value);
  if (path.empty()) {
    throw tg::NodeError("PointCloudReader: path is empty");
  }
  pcl::PCLPointCloud2 blob;
  if (!LoadCloud2(path, blob)) {
    throw tg::NodeError("PointCloudReader: failed to read '" + path + "'");
  }
  auto cloud = desc->from_cloud2(blob);
  if (cloud == nullptr) {
    throw tg::NodeError("PointCloudReader: failed to convert '" + path + "'");
  }
  _out = Wrap(std::move(cloud), desc);
}

PointCloudWriterNode::PointCloudWriterNode()
    : inport{tg::In<CloudData>{this, kInIndex}} {
  setTypeId(kTypeId);
  _types = AllTypes();
  _path = addProperty("path", std::string(), FileHints());
  _format = addProperty("format", 0,
                        EnumHints({"PCD (ascii)", "PCD (binary)", "PLY"}));
  _input_type = addProperty("input_type", 0,
                            PointTypeHints(_types, false));
  _input_type_index = static_cast<int>(getProperties().size()) - 1;
}

void PointCloudWriterNode::setPath(std::string path) {
  _path->value = std::move(path);
}

void PointCloudWriterNode::setFormat(int format) {
  _format->value = format;
}

void PointCloudWriterNode::setInputType(int index) {
  _input_type->value = index;
  refreshPortTypes();
}

const PointTypeDesc* PointCloudWriterNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_input_type->value), _types, false);
}

tg::NodeDataType PointCloudWriterNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::In) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

int PointCloudWriterNode::portTypeProperty(tg::PortType port_type,
                                           unsigned int index) const {
  return port_type == tg::PortType::In && index == kInIndex
             ? _input_type_index
             : -1;
}

void PointCloudWriterNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("PointCloudWriter: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("PointCloudWriter: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("PointCloudWriter: input point type mismatch");
  }
  const std::string path = std::get<std::string>(_path->value);
  if (path.empty()) {
    throw tg::NodeError("PointCloudWriter: path is empty");
  }

  pcl::PCLPointCloud2 blob;
  desc->to_cloud2(cloud->cloud(), blob);

  int result = 0;
  switch (std::get<int>(_format->value)) {
    case 0: {
      pcl::PCDWriter writer;
      result = writer.write(path, blob, Eigen::Vector4f::Zero(),
                            Eigen::Quaternionf::Identity(), false);
      break;
    }
    case 1: {
      pcl::PCDWriter writer;
      result = writer.write(path, blob, Eigen::Vector4f::Zero(),
                            Eigen::Quaternionf::Identity(), true);
      break;
    }
    default: {
      pcl::PLYWriter writer;
      result = writer.write(path, blob, Eigen::Vector4f::Zero(),
                            Eigen::Quaternionf::Identity(), false, true);
      break;
    }
  }
  if (result != 0) {
    throw tg::NodeError("PointCloudWriter: failed to write '" + path + "'");
  }
}

}  // namespace pg
