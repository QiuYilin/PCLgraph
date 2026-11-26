#pragma once

#include <memory>
#include <string>
#include <vector>

#include <taskgraph/Node.hpp>

#include "pclgraph/double_data.hpp"
#include "pclgraph/nodes/pcl_node_base.hpp"
#include "pclgraph/point_type.hpp"
#include "pclgraph/transform_data.hpp"

namespace pg {

/** \brief 读 PCD/PLY；Auto 时按文件头字段推断点类型。 */
class PointCloudReaderNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.io.reader";
  static constexpr unsigned int kOutIndex = 0;

  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  PointCloudReaderNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::Out ? 1u : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override {
    return port_type == tg::PortType::Out && index == kOutIndex ? "cloud" : "";
  }

  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }

  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int, std::shared_ptr<tg::NodeData>) override {}

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPath(std::string path);
  void setPointType(int index);

  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;
  const PointTypeDesc* ResolveAuto() const;

  std::shared_ptr<Property> _path;
  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::vector<const PointTypeDesc*> _types;
  mutable std::string _cache_path;
  mutable const PointTypeDesc* _cache_desc = nullptr;
  mutable bool _cache_valid = false;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 按输入点类型写 PCD/PLY。 */
class PointCloudWriterNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.io.writer";
  static constexpr unsigned int kInIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };

  PointCloudWriterNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In ? 1u : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override {
    return port_type == tg::PortType::In && index == kInIndex ? "cloud" : "";
  }

  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }

  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;

  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int) override {
    return nullptr;
  }

  void setPath(std::string path);
  void setFormat(int format);
  void setInputType(int index);

  InPort inport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _path;
  std::shared_ptr<Property> _format;
  std::shared_ptr<Property> _input_type;
  int _input_type_index = -1;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
};

/** \brief 体素下采样。 */
class VoxelGridNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.voxel_grid";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  VoxelGridNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;

  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }

  void compute() override;

  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void setLeafSize(double x, double y, double z);
  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _leaf_x;
  std::shared_ptr<Property> _leaf_y;
  std::shared_ptr<Property> _leaf_z;
  std::shared_ptr<Property> _min_points;
  std::shared_ptr<Property> _field;
  std::shared_ptr<Property> _limits_enabled;
  std::shared_ptr<Property> _limit_min;
  std::shared_ptr<Property> _limit_max;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 单字段直通滤波。 */
class PassThroughNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.passthrough";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  PassThroughNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;

  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }

  void compute() override;

  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _field;
  std::shared_ptr<Property> _limit_min;
  std::shared_ptr<Property> _limit_max;
  std::shared_ptr<Property> _negative;
  std::shared_ptr<Property> _keep_organized;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 法线估计；输出类型由输入类型映射（XYZ→PointNormal 等）。 */
class NormalEstimationNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.features.normal_estimation";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> normals;
  };

  NormalEstimationNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;

  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }

  void compute() override;

  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;
  const PointTypeDesc* OutputType() const;

  std::shared_ptr<Property> _input_type;
  int _input_type_index = -1;
  std::shared_ptr<Property> _k_search;
  std::shared_ptr<Property> _radius_search;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 显式点类型转换；跨类型连线的唯一入口。 */
class PointCloudConvertNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.tool.convert";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> in;
  };
  struct OutPort {
    tg::Out<CloudData> out;
  };

  PointCloudConvertNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;

  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }

  void compute() override;

  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* InputType() const;
  const PointTypeDesc* OutputType() const;

  std::shared_ptr<Property> _input_type;
  int _input_type_index = -1;
  std::shared_ptr<Property> _output_type;
  int _output_type_index = -1;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 中值滤波；要求 organized 点云。 */
class MedianFilterNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.median_filter";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  MedianFilterNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _window_size;
  std::shared_ptr<Property> _max_allowed_movement;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 半径离群点移除。 */
class RadiusOutlierRemovalNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.radius_outlier_removal";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  RadiusOutlierRemovalNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _search_radius;
  std::shared_ptr<Property> _min_neighbors;
  std::shared_ptr<Property> _negative;
  std::shared_ptr<Property> _keep_organized;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 轴对齐/可旋转盒子裁剪。 */
class CropBoxNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.crop_box";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  CropBoxNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _min_x;
  std::shared_ptr<Property> _min_y;
  std::shared_ptr<Property> _min_z;
  std::shared_ptr<Property> _max_x;
  std::shared_ptr<Property> _max_y;
  std::shared_ptr<Property> _max_z;
  std::shared_ptr<Property> _rotation_x;
  std::shared_ptr<Property> _rotation_y;
  std::shared_ptr<Property> _rotation_z;
  std::shared_ptr<Property> _translation_x;
  std::shared_ptr<Property> _translation_y;
  std::shared_ptr<Property> _translation_z;
  std::shared_ptr<Property> _keep_organized;
  std::shared_ptr<Property> _negative;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 近似体素下采样。 */
class ApproximateVoxelGridNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId =
      "pcl.filter.approximate_voxel_grid";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  ApproximateVoxelGridNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _leaf_x;
  std::shared_ptr<Property> _leaf_y;
  std::shared_ptr<Property> _leaf_z;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 均匀采样。 */
class UniformSamplingNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.uniform_sampling";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  UniformSamplingNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _search_radius;
  std::shared_ptr<Property> _min_points_per_voxel;
  std::shared_ptr<Property> _negative;
  std::shared_ptr<Property> _keep_organized;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 移除 NaN/Inf 点。 */
class RemoveNanNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.remove_nan";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  RemoveNanNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 双边滤波；类型跟随 PCL 预编译列表（XYZI / XYZINormal）。 */
class BilateralFilterNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.bilateral_filter";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  BilateralFilterNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _sigma_s;
  std::shared_ptr<Property> _sigma_r;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 快速双边滤波；要求 organized，类型跟随 PCL 预编译列表。 */
class FastBilateralFilterNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.fast_bilateral_filter";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  FastBilateralFilterNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _sigma_s;
  std::shared_ptr<Property> _sigma_r;
  std::shared_ptr<Property> _threads;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 阴影点移除；输入主点云 + 法线。 */
class ShadowPointsNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.shadow_points";
  static constexpr unsigned int kInCloudIndex = 0;
  static constexpr unsigned int kInNormalsIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
    tg::In<CloudData> normals;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  ShadowPointsNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _threshold;
  std::shared_ptr<Property> _keep_organized;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_cloud;
  std::weak_ptr<tg::NodeData> _in_normals;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 协方差采样；输入主点云 + 法线。 */
class CovarianceSamplingNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.filter.covariance_sampling";
  static constexpr unsigned int kInCloudIndex = 0;
  static constexpr unsigned int kInNormalsIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
    tg::In<CloudData> normals;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  CovarianceSamplingNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _number_samples;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_cloud;
  std::weak_ptr<tg::NodeData> _in_normals;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief FPFH 特征；输入主点云 + 法线，输出 FPFHSignature33。 */
class FpfhEstimationNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.features.fpfh";
  static constexpr unsigned int kInCloudIndex = 0;
  static constexpr unsigned int kInNormalsIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
    tg::In<CloudData> normals;
  };
  struct OutPort {
    tg::Out<CloudData> signature;
  };

  FpfhEstimationNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _k;
  std::shared_ptr<Property> _bins_f1;
  std::shared_ptr<Property> _bins_f2;
  std::shared_ptr<Property> _bins_f3;
  std::shared_ptr<Property> _threads;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_cloud;
  std::weak_ptr<tg::NodeData> _in_normals;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief ICP Point-to-Point；输出对齐点云 + 变换矩阵。 */
class IcpPointToPointNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId =
      "pcl.registration.icp_point_to_point";
  static constexpr unsigned int kInSourceIndex = 0;
  static constexpr unsigned int kInTargetIndex = 1;
  static constexpr unsigned int kOutAlignedIndex = 0;
  static constexpr unsigned int kOutTransformIndex = 1;

  struct InPort {
    tg::In<CloudData> source;
    tg::In<CloudData> target;
  };
  struct OutPort {
    tg::Out<CloudData> aligned;
    tg::Out<TransformData> transform;
  };

  IcpPointToPointNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override;

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _max_iterations;
  std::shared_ptr<Property> _max_correspondence_distance;
  std::shared_ptr<Property> _transformation_epsilon;
  std::shared_ptr<Property> _euclidean_fitness_epsilon;
  std::shared_ptr<Property> _use_reciprocal;
  std::shared_ptr<Property> _threads;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_source;
  std::weak_ptr<tg::NodeData> _in_target;
  std::shared_ptr<tg::NodeData> _out;
  std::shared_ptr<TransformData> _transform_out;
};

/** \brief NDT 配准；输出变换矩阵。 */
class NdtNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.registration.ndt";
  static constexpr unsigned int kInSourceIndex = 0;
  static constexpr unsigned int kInTargetIndex = 1;
  static constexpr unsigned int kOutTransformIndex = 0;

  struct InPort {
    tg::In<CloudData> source;
    tg::In<CloudData> target;
  };
  struct OutPort {
    tg::Out<TransformData> transform;
  };

  NdtNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutTransformIndex ? _transform_out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _resolution;
  std::shared_ptr<Property> _min_points_per_voxel;
  std::shared_ptr<Property> _step_size;
  std::shared_ptr<Property> _max_iterations;
  std::shared_ptr<Property> _transformation_epsilon;
  std::shared_ptr<Property> _euclidean_fitness_epsilon;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_source;
  std::weak_ptr<tg::NodeData> _in_target;
  std::shared_ptr<TransformData> _transform_out;
};

/** \brief SAC-IA 初始配准；需要源/目标点云及各自的 FPFH 特征。 */
class SacIaNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.registration.sac_ia";
  static constexpr unsigned int kInSourceIndex = 0;
  static constexpr unsigned int kInSourceFeaturesIndex = 1;
  static constexpr unsigned int kInTargetIndex = 2;
  static constexpr unsigned int kInTargetFeaturesIndex = 3;
  static constexpr unsigned int kOutAlignedIndex = 0;
  static constexpr unsigned int kOutTransformIndex = 1;

  struct InPort {
    tg::In<CloudData> source;
    tg::In<CloudData> source_features;
    tg::In<CloudData> target;
    tg::In<CloudData> target_features;
  };
  struct OutPort {
    tg::Out<CloudData> aligned;
    tg::Out<TransformData> transform;
  };

  SacIaNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override;

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _transform_estimation;
  std::shared_ptr<Property> _max_iterations;
  std::shared_ptr<Property> _min_sample_distance;
  std::shared_ptr<Property> _nr_samples;
  std::shared_ptr<Property> _k_correspondences;
  std::shared_ptr<Property> _error_functor;
  std::shared_ptr<Property> _error_threshold;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_source;
  std::weak_ptr<tg::NodeData> _in_source_features;
  std::weak_ptr<tg::NodeData> _in_target;
  std::weak_ptr<tg::NodeData> _in_target_features;
  std::shared_ptr<tg::NodeData> _out;
  std::shared_ptr<TransformData> _transform_out;
};

/** \brief 用 Transform 变换点云。 */
class TransformPointCloudNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId =
      "pcl.tool.transform_point_cloud";
  static constexpr unsigned int kInCloudIndex = 0;
  static constexpr unsigned int kInTransformIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
    tg::In<TransformData> transform;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  TransformPointCloudNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_cloud;
  std::weak_ptr<tg::NodeData> _in_transform;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 主点云 + Normal 合并成带法线的点类型。 */
class ConcatenateFieldsNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.io.concatenate_fields";
  static constexpr unsigned int kInCloudIndex = 0;
  static constexpr unsigned int kInNormalsIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
    tg::In<CloudData> normals;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  ConcatenateFieldsNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;
  const PointTypeDesc* OutputType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_cloud;
  std::weak_ptr<tg::NodeData> _in_normals;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 两个同类型点云拼接。 */
class PointCloudMergeNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.io.merge";
  static constexpr unsigned int kInFirstIndex = 0;
  static constexpr unsigned int kInSecondIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> first;
    tg::In<CloudData> second;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  PointCloudMergeNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_first;
  std::weak_ptr<tg::NodeData> _in_second;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 对每个点的最近非零邻距求平均；没有有效邻居的点不计入。 */
class CalcModelResoNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.tool.calc_model_reso";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<DoubleData> value;
  };

  CalcModelResoNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _value_out : nullptr;
  }

  InPort inport;
  OutPort outport;

 private:
  std::shared_ptr<Property> _k;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<DoubleData> _value_out;
};

/** \brief SIFT keypoint；输入 PointXYZI，输出 PointXYZ。 */
class SiftKeypointNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.keypoints.sift";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> keypoints;
  };

  SiftKeypointNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  InPort inport;
  OutPort outport;

 private:
  std::shared_ptr<Property> _min_scale;
  std::shared_ptr<Property> _nr_octaves;
  std::shared_ptr<Property> _nr_scales_per_octave;
  std::shared_ptr<Property> _min_contrast;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief ISS3D keypoint；输入/输出 PointXYZ。 */
class IssKeypoint3DNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.keypoints.iss_3d";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
  };
  struct OutPort {
    tg::Out<CloudData> keypoints;
  };

  IssKeypoint3DNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  InPort inport;
  OutPort outport;

 private:
  std::shared_ptr<Property> _salient_radius;
  std::shared_ptr<Property> _non_max_radius;
  std::shared_ptr<Property> _normal_radius;
  std::shared_ptr<Property> _border_radius;
  std::shared_ptr<Property> _gamma_21;
  std::shared_ptr<Property> _gamma_32;
  std::shared_ptr<Property> _min_neighbors;
  std::shared_ptr<Property> _threads;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief Harris3D keypoint；输入 PointXYZ + Normal，输出 PointXYZI。 */
class HarrisKeypoint3DNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.keypoints.harris_3d";
  static constexpr unsigned int kInCloudIndex = 0;
  static constexpr unsigned int kInNormalsIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
    tg::In<CloudData> normals;
  };
  struct OutPort {
    tg::Out<CloudData> keypoints;
  };

  HarrisKeypoint3DNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  InPort inport;
  OutPort outport;

 private:
  std::shared_ptr<Property> _method;
  std::shared_ptr<Property> _radius;
  std::shared_ptr<Property> _threshold;
  std::shared_ptr<Property> _non_max_suppression;
  std::shared_ptr<Property> _refine;
  std::shared_ptr<Property> _threads;
  std::weak_ptr<tg::NodeData> _in_cloud;
  std::weak_ptr<tg::NodeData> _in_normals;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief 按法线方向空间采样；输入点云 + Normal。 */
class NormalSpaceSamplingNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId =
      "pcl.filter.normal_space_sampling";
  static constexpr unsigned int kInCloudIndex = 0;
  static constexpr unsigned int kInNormalsIndex = 1;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<CloudData> cloud;
    tg::In<CloudData> normals;
  };
  struct OutPort {
    tg::Out<CloudData> cloud;
  };

  NormalSpaceSamplingNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _sample;
  std::shared_ptr<Property> _seed;
  std::shared_ptr<Property> _bins_x;
  std::shared_ptr<Property> _bins_y;
  std::shared_ptr<Property> _bins_z;
  std::shared_ptr<Property> _keep_organized;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_cloud;
  std::weak_ptr<tg::NodeData> _in_normals;
  std::shared_ptr<tg::NodeData> _out;
};

/** \brief FPCS 初始配准；输出对齐点云 + Transform。 */
class FpcsNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId = "pcl.registration.fpcs";
  static constexpr unsigned int kInSourceIndex = 0;
  static constexpr unsigned int kInTargetIndex = 1;
  static constexpr unsigned int kInSourceNormalsIndex = 2;
  static constexpr unsigned int kInTargetNormalsIndex = 3;
  static constexpr unsigned int kOutAlignedIndex = 0;
  static constexpr unsigned int kOutTransformIndex = 1;

  struct InPort {
    tg::In<CloudData> source;
    tg::In<CloudData> target;
    tg::In<CloudData> source_normals;
    tg::In<CloudData> target_normals;
  };
  struct OutPort {
    tg::Out<CloudData> aligned;
    tg::Out<TransformData> transform;
  };

  FpcsNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override;

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _ransac_iterations;
  std::shared_ptr<Property> _delta;
  std::shared_ptr<Property> _approx_overlap;
  std::shared_ptr<Property> _score_threshold;
  std::shared_ptr<Property> _nr_samples;
  std::shared_ptr<Property> _use_normals;
  std::shared_ptr<Property> _max_norm_diff;
  std::shared_ptr<Property> _max_runtime;
  std::shared_ptr<Property> _threads;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_source;
  std::weak_ptr<tg::NodeData> _in_target;
  std::weak_ptr<tg::NodeData> _in_source_normals;
  std::weak_ptr<tg::NodeData> _in_target_normals;
  std::shared_ptr<tg::NodeData> _out;
  std::shared_ptr<TransformData> _transform_out;
};

/** \brief ICP Point-to-Plane；输入 PointNormal / PointXYZINormal。 */
class IcpPointToPlaneNode final : public PclNodeBase {
 public:
  static constexpr const char* kTypeId =
      "pcl.registration.icp_point_to_plane";
  static constexpr unsigned int kInSourceIndex = 0;
  static constexpr unsigned int kInTargetIndex = 1;
  static constexpr unsigned int kOutAlignedIndex = 0;
  static constexpr unsigned int kOutTransformIndex = 1;

  struct InPort {
    tg::In<CloudData> source;
    tg::In<CloudData> target;
  };
  struct OutPort {
    tg::Out<CloudData> aligned;
    tg::Out<TransformData> transform;
  };

  IcpPointToPlaneNode();

  unsigned int nFixedPorts(tg::PortType port_type) const override;
  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override;
  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override;
  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }
  int portTypeProperty(tg::PortType port_type,
                       unsigned int index) const override;

  void compute() override;
  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override;
  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override;

  void setPointType(int index);

  InPort inport;
  OutPort outport;

 private:
  const PointTypeDesc* SelectedType() const;

  std::shared_ptr<Property> _point_type;
  int _point_type_index = -1;
  std::shared_ptr<Property> _max_iterations;
  std::shared_ptr<Property> _max_correspondence_distance;
  std::shared_ptr<Property> _transformation_epsilon;
  std::shared_ptr<Property> _rotation_epsilon;
  std::shared_ptr<Property> _euclidean_fitness_epsilon;
  std::shared_ptr<Property> _ransac_iterations;
  std::shared_ptr<Property> _inlier_threshold;
  std::shared_ptr<Property> _use_reciprocal;
  std::shared_ptr<Property> _use_symmetric_objective;
  std::shared_ptr<Property> _enforce_same_direction_normals;
  std::shared_ptr<Property> _threads;
  std::vector<const PointTypeDesc*> _types;
  std::weak_ptr<tg::NodeData> _in_source;
  std::weak_ptr<tg::NodeData> _in_target;
  std::shared_ptr<tg::NodeData> _out;
  std::shared_ptr<TransformData> _transform_out;
};

}  // namespace pg
