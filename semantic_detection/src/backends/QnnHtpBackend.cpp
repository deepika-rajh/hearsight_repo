#include "semantic_detection/backends/QnnHtpBackend.hpp"

#include <dlfcn.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace semantic_detection {

namespace {

void checkQnn(Qnn_ErrorHandle_t status, const char * what) {
  if (status != QNN_SUCCESS) {
    throw std::runtime_error(std::string("QnnHtpBackend: ") + what +
                              " failed with QNN error code " + std::to_string(status));
  }
}

size_t bytesPerElement(const QnnTensorSpec & spec) {
  if (!spec.quantized) return sizeof(float);
  return spec.bits == 8 ? sizeof(uint8_t) : sizeof(uint16_t);
}

/// QNN dequant convention: float = (quantized + offset) * scale, so
/// quantized = round(float / scale - offset), clamped to the tensor's range.
void quantizeInto(const QnnTensorSpec & spec, const float * src, size_t count, void * dst) {
  const int max_val = (1 << spec.bits) - 1;
  auto quantize_one = [&](float f) {
    const long q = std::lround(f / spec.scale - spec.offset);
    return static_cast<int>(std::clamp<long>(q, 0, max_val));
  };
  if (spec.bits == 8) {
    auto * out = static_cast<uint8_t *>(dst);
    for (size_t i = 0; i < count; ++i) out[i] = static_cast<uint8_t>(quantize_one(src[i]));
  } else {
    auto * out = static_cast<uint16_t *>(dst);
    for (size_t i = 0; i < count; ++i) out[i] = static_cast<uint16_t>(quantize_one(src[i]));
  }
}

void dequantizeInto(const QnnTensorSpec & spec, const void * src, size_t count, float * dst) {
  if (spec.bits == 8) {
    const auto * in = static_cast<const uint8_t *>(src);
    for (size_t i = 0; i < count; ++i) dst[i] = (static_cast<float>(in[i]) + spec.offset) * spec.scale;
  } else {
    const auto * in = static_cast<const uint16_t *>(src);
    for (size_t i = 0; i < count; ++i) dst[i] = (static_cast<float>(in[i]) + spec.offset) * spec.scale;
  }
}

std::vector<char> readFile(const std::string & path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("QnnHtpBackend: could not open context binary: " + path);
  }
  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> buffer(static_cast<size_t>(size));
  if (!file.read(buffer.data(), size)) {
    throw std::runtime_error("QnnHtpBackend: failed reading context binary: " + path);
  }
  return buffer;
}

// QnnSystemContext_BinaryInfo_t's numGraphs/graphs fields live at different
// offsets depending on .version (V3 drops the hwInfoBlob* fields V1/V2
// have), so this must switch on version rather than assume a common prefix.
void graphsFromBinaryInfo(const QnnSystemContext_BinaryInfo_t & info,
                          const QnnSystemContext_GraphInfo_t *& graphs, uint32_t & num_graphs) {
  switch (info.version) {
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_1:
      graphs = info.contextBinaryInfoV1.graphs;
      num_graphs = info.contextBinaryInfoV1.numGraphs;
      return;
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_2:
      graphs = info.contextBinaryInfoV2.graphs;
      num_graphs = info.contextBinaryInfoV2.numGraphs;
      return;
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_3:
      graphs = info.contextBinaryInfoV3.graphs;
      num_graphs = info.contextBinaryInfoV3.numGraphs;
      return;
    default:
      throw std::runtime_error("QnnHtpBackend: unsupported QnnSystemContext_BinaryInfo version");
  }
}

void tensorsFromGraphInfo(const QnnSystemContext_GraphInfo_t & graph, const char *& graph_name,
                          const Qnn_Tensor_t *& inputs, uint32_t & num_inputs,
                          const Qnn_Tensor_t *& outputs, uint32_t & num_outputs) {
  switch (graph.version) {
    case QNN_SYSTEM_CONTEXT_GRAPH_INFO_VERSION_1:
      graph_name = graph.graphInfoV1.graphName;
      inputs = graph.graphInfoV1.graphInputs;
      num_inputs = graph.graphInfoV1.numGraphInputs;
      outputs = graph.graphInfoV1.graphOutputs;
      num_outputs = graph.graphInfoV1.numGraphOutputs;
      return;
    case QNN_SYSTEM_CONTEXT_GRAPH_INFO_VERSION_2:
      graph_name = graph.graphInfoV2.graphName;
      inputs = graph.graphInfoV2.graphInputs;
      num_inputs = graph.graphInfoV2.numGraphInputs;
      outputs = graph.graphInfoV2.graphOutputs;
      num_outputs = graph.graphInfoV2.numGraphOutputs;
      return;
    case QNN_SYSTEM_CONTEXT_GRAPH_INFO_VERSION_3:
      graph_name = graph.graphInfoV3.graphName;
      inputs = graph.graphInfoV3.graphInputs;
      num_inputs = graph.graphInfoV3.numGraphInputs;
      outputs = graph.graphInfoV3.graphOutputs;
      num_outputs = graph.graphInfoV3.numGraphOutputs;
      return;
    default:
      throw std::runtime_error("QnnHtpBackend: unsupported QnnSystemContext_GraphInfo version");
  }
}

const char * tensorName(const Qnn_Tensor_t & tensor) {
  return tensor.version == QNN_TENSOR_VERSION_2 ? tensor.v2.name : tensor.v1.name;
}

const Qnn_Tensor_t * findTensorByName(const Qnn_Tensor_t * tensors, uint32_t count,
                                      const std::string & name) {
  for (uint32_t i = 0; i < count; ++i) {
    const char * candidate = tensorName(tensors[i]);
    if (candidate && name == candidate) {
      return &tensors[i];
    }
  }
  return nullptr;
}

/// Deep-copies a Qnn_Tensor_t retrieved from QnnSystemContext (whose
/// name/dimensions pointers point into memory owned by that system-context
/// object) into `dst`, backed by `name_storage`/`dims_storage` so it stays
/// valid after the system context is freed. Preserves id/version/type/
/// dataFormat/dataType/quantizeParams/memType/rank exactly as retrieved.
void copyTensorMetadata(const Qnn_Tensor_t & src, Qnn_Tensor_t & dst, std::string & name_storage,
                        std::vector<uint32_t> & dims_storage) {
  dst = src;

  const bool is_v2 = src.version == QNN_TENSOR_VERSION_2;
  const char * src_name = is_v2 ? src.v2.name : src.v1.name;
  const uint32_t rank = is_v2 ? src.v2.rank : src.v1.rank;
  const uint32_t * src_dims = is_v2 ? src.v2.dimensions : src.v1.dimensions;

  name_storage = src_name ? src_name : "";
  dims_storage.assign(src_dims, src_dims + rank);

  if (is_v2) {
    dst.v2.name = name_storage.c_str();
    dst.v2.dimensions = dims_storage.data();
  } else {
    dst.v1.name = name_storage.c_str();
    dst.v1.dimensions = dims_storage.data();
  }
}

void setClientBuffer(Qnn_Tensor_t & tensor, void * data, uint32_t size) {
  if (tensor.version == QNN_TENSOR_VERSION_2) {
    tensor.v2.clientBuf.data = data;
    tensor.v2.clientBuf.dataSize = size;
  } else {
    tensor.v1.clientBuf.data = data;
    tensor.v1.clientBuf.dataSize = size;
  }
}

}  // namespace

QnnHtpBackend::QnnHtpBackend(QnnHtpConfig config) : config_(std::move(config)) {
  loadBackendLibrary();
}

QnnHtpBackend::~QnnHtpBackend() { teardown(); }

void QnnHtpBackend::loadBackendLibrary() {
  backend_lib_handle_ = dlopen(config_.backend_lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!backend_lib_handle_) {
    throw std::runtime_error(std::string("QnnHtpBackend: dlopen failed for ") +
                              config_.backend_lib_path + ": " + dlerror());
  }

  using GetProvidersFn = Qnn_ErrorHandle_t (*)(const QnnInterface_t ***, uint32_t *);
  auto get_providers =
      reinterpret_cast<GetProvidersFn>(dlsym(backend_lib_handle_, "QnnInterface_getProviders"));
  if (!get_providers) {
    throw std::runtime_error("QnnHtpBackend: QnnInterface_getProviders symbol not found in " +
                              config_.backend_lib_path);
  }

  const QnnInterface_t ** providers = nullptr;
  uint32_t num_providers = 0;
  checkQnn(get_providers(&providers, &num_providers), "QnnInterface_getProviders");
  if (num_providers == 0) {
    throw std::runtime_error("QnnHtpBackend: backend library reported zero interface providers");
  }

  // Backend libraries typically expose exactly one provider matching the
  // major API version this header was compiled against.
  const QnnInterface_t * selected = nullptr;
  for (uint32_t i = 0; i < num_providers; ++i) {
    if (providers[i]->apiVersion.coreApiVersion.major == QNN_API_VERSION_MAJOR) {
      selected = providers[i];
      break;
    }
  }
  if (!selected) {
    selected = providers[0];
  }
  qnn_ = &selected->QNN_INTERFACE_VER_NAME;
}

void QnnHtpBackend::loadSystemLibrary() {
  system_lib_handle_ = dlopen("libQnnSystem.so", RTLD_NOW | RTLD_LOCAL);
  if (!system_lib_handle_) {
    throw std::runtime_error(std::string("QnnHtpBackend: dlopen failed for libQnnSystem.so: ") +
                              dlerror());
  }

  using GetProvidersFn = Qnn_ErrorHandle_t (*)(const QnnSystemInterface_t ***, uint32_t *);
  auto get_providers = reinterpret_cast<GetProvidersFn>(
      dlsym(system_lib_handle_, "QnnSystemInterface_getProviders"));
  if (!get_providers) {
    throw std::runtime_error(
        "QnnHtpBackend: QnnSystemInterface_getProviders symbol not found in libQnnSystem.so");
  }

  const QnnSystemInterface_t ** providers = nullptr;
  uint32_t num_providers = 0;
  checkQnn(get_providers(&providers, &num_providers), "QnnSystemInterface_getProviders");
  if (num_providers == 0) {
    throw std::runtime_error("QnnHtpBackend: libQnnSystem.so reported zero interface providers");
  }

  const QnnSystemInterface_t * selected = nullptr;
  for (uint32_t i = 0; i < num_providers; ++i) {
    if (providers[i]->systemApiVersion.major == QNN_SYSTEM_API_VERSION_MAJOR) {
      selected = providers[i];
      break;
    }
  }
  if (!selected) {
    selected = providers[0];
  }
  qnn_system_ = &selected->QNN_SYSTEM_INTERFACE_VER_NAME;
}

void QnnHtpBackend::retrieveGraphTensors(const std::vector<char> & context_binary) {
  loadSystemLibrary();

  QnnSystemContext_Handle_t sys_ctx = nullptr;
  checkQnn(qnn_system_->systemContextCreate(&sys_ctx), "systemContextCreate");

  const QnnSystemContext_BinaryInfo_t * binary_info = nullptr;
  Qnn_ContextBinarySize_t binary_info_size = 0;
  const Qnn_ErrorHandle_t get_info_status = qnn_system_->systemContextGetBinaryInfo(
      sys_ctx, const_cast<void *>(static_cast<const void *>(context_binary.data())),
      static_cast<uint64_t>(context_binary.size()), &binary_info, &binary_info_size);
  if (get_info_status != QNN_SUCCESS) {
    qnn_system_->systemContextFree(sys_ctx);
    checkQnn(get_info_status, "systemContextGetBinaryInfo");
  }

  const QnnSystemContext_GraphInfo_t * graphs = nullptr;
  uint32_t num_graphs = 0;
  graphsFromBinaryInfo(*binary_info, graphs, num_graphs);

  const QnnSystemContext_GraphInfo_t * target_graph = nullptr;
  for (uint32_t i = 0; i < num_graphs; ++i) {
    const char * graph_name = nullptr;
    const Qnn_Tensor_t * inputs = nullptr;
    uint32_t num_inputs = 0;
    const Qnn_Tensor_t * outputs = nullptr;
    uint32_t num_outputs = 0;
    tensorsFromGraphInfo(graphs[i], graph_name, inputs, num_inputs, outputs, num_outputs);
    if (graph_name && config_.graph_name == graph_name) {
      target_graph = &graphs[i];
      break;
    }
  }
  if (!target_graph) {
    qnn_system_->systemContextFree(sys_ctx);
    throw std::runtime_error("QnnHtpBackend: graph '" + config_.graph_name +
                             "' not found in context binary metadata");
  }

  const char * graph_name = nullptr;
  const Qnn_Tensor_t * inputs = nullptr;
  uint32_t num_inputs = 0;
  const Qnn_Tensor_t * outputs = nullptr;
  uint32_t num_outputs = 0;
  tensorsFromGraphInfo(*target_graph, graph_name, inputs, num_inputs, outputs, num_outputs);

  const Qnn_Tensor_t * input_src = findTensorByName(inputs, num_inputs, config_.input.name);
  const Qnn_Tensor_t * output_src = findTensorByName(outputs, num_outputs, config_.output.name);
  if (!input_src || !output_src) {
    qnn_system_->systemContextFree(sys_ctx);
    throw std::runtime_error("QnnHtpBackend: tensor '" +
                             (input_src ? config_.output.name : config_.input.name) +
                             "' not found among graph '" + config_.graph_name + "'s " +
                             (input_src ? "outputs" : "inputs") + " in the context binary metadata");
  }

  copyTensorMetadata(*input_src, input_tensor_, input_tensor_name_storage_, input_dims_storage_);
  copyTensorMetadata(*output_src, output_tensor_, output_tensor_name_storage_, output_dims_storage_);

  qnn_system_->systemContextFree(sys_ctx);
}

void QnnHtpBackend::loadModel(const std::string & model_path) {
  checkQnn(qnn_->backendCreate(nullptr, nullptr, &backend_handle_), "backendCreate");
  checkQnn(qnn_->deviceCreate(nullptr, nullptr, &device_handle_), "deviceCreate");

  const std::vector<char> context_binary = readFile(model_path);
  checkQnn(qnn_->contextCreateFromBinary(backend_handle_, device_handle_, nullptr,
                                         context_binary.data(),
                                         static_cast<Qnn_ContextBinarySize_t>(context_binary.size()),
                                         &context_handle_, nullptr),
           "contextCreateFromBinary");

  checkQnn(qnn_->graphRetrieve(context_handle_, config_.graph_name.c_str(), &graph_handle_),
           "graphRetrieve");

  // Grabs the graph's own tensor descriptors (real IDs included) from the
  // same binary bytes -- graphExecute() validates tensor IDs, so we must
  // execute with these, not freshly-constructed Qnn_Tensor_t objects.
  retrieveGraphTensors(context_binary);
}

RawOutput QnnHtpBackend::infer(const cv::Mat & letterboxed_bgr) {
  cv::Mat rgb;
  cv::cvtColor(letterboxed_bgr, rgb, cv::COLOR_BGR2RGB);
  if (!rgb.isContinuous()) {
    rgb = rgb.clone();
  }

  if (!config_.input.quantized) {
    throw std::runtime_error(
        "QnnHtpBackend: float32 graph input is not implemented -- add the float "
        "preprocessing your existing pipeline uses here.");
  }

  // Pixels are HWC uint8 [0,255]; the graph's calibration was done on
  // [0,1]-normalized float images (per model_info.py: scale ~= 1/65536,
  // offset 0), so normalize before quantizing -- do NOT feed raw 0-255
  // values into quantizeInto() directly.
  const size_t num_pixels = rgb.total() * static_cast<size_t>(rgb.channels());
  std::vector<float> normalized(num_pixels);
  for (size_t i = 0; i < num_pixels; ++i) {
    normalized[i] = static_cast<float>(rgb.data[i]) / 255.0f;
  }

  std::vector<uint8_t> input_buffer(num_pixels * bytesPerElement(config_.input));
  quantizeInto(config_.input, normalized.data(), num_pixels, input_buffer.data());

  const uint32_t output_num_elements = [&] {
    uint32_t n = 1;
    for (auto d : config_.output.dims) n *= d;
    return n;
  }();
  std::vector<uint8_t> output_buffer(output_num_elements * bytesPerElement(config_.output));

  // input_tensor_/output_tensor_ are the graph's own tensor descriptors
  // (retrieved once in loadModel(), real IDs/dims/quantizeParams intact) --
  // only their buffer pointers change per call, no new Qnn_Tensor_t objects.
  setClientBuffer(input_tensor_, input_buffer.data(), static_cast<uint32_t>(input_buffer.size()));
  setClientBuffer(output_tensor_, output_buffer.data(), static_cast<uint32_t>(output_buffer.size()));

  checkQnn(
      qnn_->graphExecute(graph_handle_, &input_tensor_, 1, &output_tensor_, 1, nullptr, nullptr),
      "graphExecute");

  RawOutput result;
  // Ultralytics-style head output: channel 0..3 = box, 4.. = class scores.
  // config_.output.dims is expected to describe [1, num_channels, num_boxes]
  // (or the equivalent for your graph) -- adjust here if your compiled
  // graph's output layout differs.
  if (config_.output.dims.size() != 3) {
    throw std::runtime_error(
        "QnnHtpBackend: expected a rank-3 output tensor [1, num_channels, num_boxes]");
  }
  result.num_channels = static_cast<int>(config_.output.dims[1]);
  result.num_boxes = static_cast<int>(config_.output.dims[2]);
  result.data.resize(output_num_elements);
  if (config_.output.quantized) {
    dequantizeInto(config_.output, output_buffer.data(), output_num_elements, result.data.data());
  } else {
    std::memcpy(result.data.data(), output_buffer.data(), output_buffer.size());
  }
  return result;
}

void QnnHtpBackend::teardown() {
  if (qnn_ && context_handle_) {
    qnn_->contextFree(context_handle_, nullptr);
    context_handle_ = nullptr;
  }
  if (qnn_ && device_handle_) {
    qnn_->deviceFree(device_handle_);
    device_handle_ = nullptr;
  }
  if (qnn_ && backend_handle_) {
    qnn_->backendFree(backend_handle_);
    backend_handle_ = nullptr;
  }
  if (backend_lib_handle_) {
    dlclose(backend_lib_handle_);
    backend_lib_handle_ = nullptr;
  }
  if (system_lib_handle_) {
    dlclose(system_lib_handle_);
    system_lib_handle_ = nullptr;
  }
}

}  // namespace semantic_detection
