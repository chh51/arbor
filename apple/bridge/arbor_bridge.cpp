/**
 * @file arbor_bridge.cpp
 * Project: Arbor
 *
 * Copyright (C) 2026 Jean-Romain Roussel (r-lidar) <info @ r-lidar.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "arbor_bridge.h"

#include "arbor.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

using arbor::qsm::QSM;
using arbor::qsm::QSF;
using arbor::qsm::QSMEdge;
using arbor::qsm::QSMNode;

void arbor_bridge_attributes_free(ArborBridgeAttributes *value)
{
  if (!value) return;
  free(value->order);
  free(value->classification);
  free(value->hag);
  free(value->foliage);
  free(value->passage);
  free(value->user_data);
  free(value->tree_id);
  free(value->red);
  free(value->green);
  free(value->blue);
  *value = {};
}

void arbor_bridge_seeds_free(ArborBridgeSeedCloud *value)
{
  if (!value) return;
  free(value->x);
  free(value->y);
  free(value->z);
  free(value->tree_id);
  free(value->foliage);
  free(value->hag);
  free(value->pwood);
  free(value->passage);
  *value = {};
}

void arbor_bridge_qsm_free(ArborBridgeQSM *value)
{
  if (!value) return;
  free(value->name);
  free(value->crs);
  if (value->messages)
  {
    for (size_t i = 0; i < value->message_count; ++i) free(value->messages[i]);
    free(value->messages);
  }
  free(value->cylinders);
  *value = {};
}

void arbor_bridge_qsf_free(ArborBridgeQSF *value)
{
  if (!value) return;
  if (value->models)
  {
    for (size_t i = 0; i < value->count; ++i) arbor_bridge_qsm_free(&value->models[i]);
    free(value->models);
  }
  *value = {};
}

void arbor_bridge_string_free(char *value)
{
  free(value);
}

namespace {

enum class OrderSlot { None, TreeID, Passage };

template <typename F>
int guard(char **error, F&& body)
{
  if (error) *error = nullptr;
  try
  {
    body();
    return 0;
  }
  catch (const std::exception& e)
  {
    if (error) *error = strdup(e.what() ? e.what() : "unknown error");
    return 1;
  }
  catch (...)
  {
    if (error) *error = strdup("unknown error");
    return 1;
  }
}

template <typename T>
T* alloc_n(size_t n)
{
  if (n == 0) return nullptr;
  T* out = static_cast<T*>(malloc(n * sizeof(T)));
  if (!out) throw std::bad_alloc();
  return out;
}

char* dup_string(const std::string& text)
{
  char* out = static_cast<char*>(malloc(text.size() + 1));
  if (!out) throw std::bad_alloc();
  memcpy(out, text.c_str(), text.size() + 1);
  return out;
}

uint8_t to_u8(uint16_t value)
{
  return value > 255 ? static_cast<uint8_t>(value >> 8) : static_cast<uint8_t>(value);
}

double shown(float value, float unset)
{
  return value == unset ? std::numeric_limits<double>::quiet_NaN() : static_cast<double>(value);
}

float stored(double value, float unset)
{
  return std::isnan(value) ? unset : static_cast<float>(value);
}

struct AttributeHolder {
  ArborBridgeAttributes value{};
  ~AttributeHolder() { arbor_bridge_attributes_free(&value); }
  ArborBridgeAttributes release()
  {
    ArborBridgeAttributes out = value;
    value = {};
    return out;
  }
};

struct SeedHolder {
  ArborBridgeSeedCloud value{};
  ~SeedHolder() { arbor_bridge_seeds_free(&value); }
  ArborBridgeSeedCloud release()
  {
    ArborBridgeSeedCloud out = value;
    value = {};
    return out;
  }
};

struct QSMHolder {
  ArborBridgeQSM value{};
  ~QSMHolder() { arbor_bridge_qsm_free(&value); }
  ArborBridgeQSM release()
  {
    ArborBridgeQSM out = value;
    value = {};
    return out;
  }
};

struct QSFHolder {
  ArborBridgeQSF value{};
  ~QSFHolder() { arbor_bridge_qsf_free(&value); }
  ArborBridgeQSF release()
  {
    ArborBridgeQSF out = value;
    value = {};
    return out;
  }
};

arbor::settings::ArborParameters make_params(const ArborBridgeParams& in)
{
  arbor::settings::ArborParameters out;
  out.global.cut_above_ground = in.cut_above_ground;
  out.woodlikelihood.k = in.wood_k;
  out.pathfinder.downward = in.downward != 0;
  out.pathfinder.k = in.graph_k;
  out.pathfinder.k_seed = in.k_seed;
  out.pathfinder.decimation = in.decimation;
  out.pathfinder.space_res = in.space_res;
  out.pathfinder.max_gap = in.max_gap;
  out.pathfinder.power = in.power;
  out.pathfinder.wood2wood = in.wood_to_wood;
  out.pathfinder.leaf2leaf = in.leaf_to_leaf;
  out.pathfinder.wood2leaf = in.wood_to_leaf;
  if (in.angle_penalty && in.angle_penalty_count > 0)
  {
    int n = in.angle_penalty_count;
    if (n > static_cast<int>(out.pathfinder.angle_penalty.size()))
      n = static_cast<int>(out.pathfinder.angle_penalty.size());
    for (int i = 0; i < n; ++i) out.pathfinder.angle_penalty[i] = in.angle_penalty[i];
  }

  out.semantic.min_passage = in.min_passage;
  out.semantic.high_pwood_threshold = in.high_pwood_threshold;
  out.semantic.medium_pwood_threshold = in.medium_pwood_threshold;
  out.semantic.connected_components_res = in.connected_components_res;
  out.semantic.connected_components_min = in.connected_components_min;
  out.semantic.wood_assignation_k = in.wood_assignation_k;
  out.semantic.wood_assignation_dist = in.wood_assignation_dist;
  out.semantic.wood_extra_reasignation_k = in.wood_extra_reasignation_k;
  out.semantic.wood_extra_reasignation_dist = in.wood_extra_reasignation_dist;
  out.semantic.medium_pwood_sor_k = in.medium_pwood_sor_k;
  out.semantic.medium_pwood_sor_m = in.medium_pwood_sor_m;
  out.semantic.ground_res = in.ground_res;

  if (in.slice_at)
    out.seeds.slice_at.assign(in.slice_at, in.slice_at + in.slice_at_count);
  out.seeds.slice_thickness = in.slice_thickness;
  out.seeds.min_passage = in.seed_min_passage;
  out.seeds.safe_zone = in.safe_zone;
  out.instance.oversegmentation_solver_enabled = in.oversegmentation_solver_enabled != 0;

  out.qsm.skeleton_node_distance = in.skeleton_node_distance;
  out.qsm.dbscan_eps_distance = in.dbscan_eps_distance;
  out.qsm.max_d = in.max_d;
  out.qsm.apex_radius = in.apex_radius;
  out.qsm.smooth_steps = in.smooth_steps;
  out.qsm.min_measurable_dbh = in.min_measurable_dbh;
  out.qsm.min_measurable_radius = in.min_measurable_radius;
  out.qsm.broken_detection_enabled = in.broken_detection_enabled != 0;
  out.qsm.allometry_scale = in.allometry_scale;
  if (in.allometry_name) out.qsm.allometry_name = in.allometry_name;
  return out;
}

PointCloud load(const ArborBridgeCloud& in, OrderSlot slot, bool force_treeid = false)
{
  if (in.count > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
    throw std::runtime_error("point cloud is too large");
  if (in.count > 0 && (!in.x || !in.y || !in.z))
    throw std::runtime_error("point cloud is missing coordinates");

  PointCloud pc(in.count, true);
  const bool keep_rgb = in.red && in.green && in.blue;
  pc.retain_attributes(
    force_treeid || slot == OrderSlot::TreeID || in.tree_id != nullptr,
    in.foliage != nullptr,
    in.classification != nullptr,
    slot == OrderSlot::Passage || in.passage != nullptr,
    in.hag != nullptr,
    in.pwood != nullptr,
    keep_rgb,
    in.user_data != nullptr);

  for (size_t i = 0; i < in.count; ++i)
  {
    pc.set_x(i, in.x[i]);
    pc.set_y(i, in.y[i]);
    pc.set_z(i, in.z[i]);
    if (in.classification) pc.set_classification(i, in.classification[i]);
    if (in.hag) pc.set_hag(i, in.hag[i]);
    if (in.pwood) pc.set_pwood(i, in.pwood[i]);
    if (in.foliage) pc.set_foliage(i, in.foliage[i]);
    if (in.passage) pc.set_passage(i, in.passage[i]);
    if (in.user_data) pc.set_userdata(i, in.user_data[i]);
    if (in.tree_id) pc.set_treeid(i, in.tree_id[i]);
    if (keep_rgb) pc.set_color(i, to_u8(in.red[i]), to_u8(in.green[i]), to_u8(in.blue[i]));
  }

  if (force_treeid)
  {
    for (size_t i = 0; i < in.count; ++i) pc.set_treeid(i, -1);
  }
  else if (slot == OrderSlot::TreeID)
  {
    for (size_t i = 0; i < in.count; ++i) pc.set_treeid(i, static_cast<int>(i));
  }
  else if (slot == OrderSlot::Passage)
  {
    for (size_t i = 0; i < in.count; ++i) pc.set_passage(i, static_cast<int>(i));
  }
  return pc;
}

void set_order(AttributeHolder& holder, const PointCloud& pc, OrderSlot slot)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  if (slot == OrderSlot::None || n == 0) return;

  int32_t* order = alloc_n<int32_t>(n);
  bool moved = false;
  for (size_t i = 0; i < n; ++i)
  {
    const int value = slot == OrderSlot::TreeID ? pc.get_treeid(i) : pc.get_passage(i);
    order[i] = value;
    if (value != static_cast<int32_t>(i)) moved = true;
  }
  if (!moved)
  {
    free(order);
    return;
  }
  holder.value.order = order;
  holder.value.reordered = 1;
}

void copy_classification(AttributeHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.classification_set = 1;
  holder.value.classification = alloc_n<int32_t>(n);
  for (size_t i = 0; i < n; ++i) holder.value.classification[i] = pc.get_classification(i);
}

void copy_hag(AttributeHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.hag_set = 1;
  holder.value.hag = alloc_n<double>(n);
  for (size_t i = 0; i < n; ++i) holder.value.hag[i] = pc.get_hag(i);
}

void copy_foliage(AttributeHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.foliage_set = 1;
  holder.value.foliage = alloc_n<int32_t>(n);
  for (size_t i = 0; i < n; ++i) holder.value.foliage[i] = pc.get_foliage(i);
}

void copy_passage(AttributeHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.passage_set = 1;
  holder.value.passage = alloc_n<int32_t>(n);
  for (size_t i = 0; i < n; ++i) holder.value.passage[i] = pc.get_passage(i);
}

void copy_userdata(AttributeHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.user_data_set = 1;
  holder.value.user_data = alloc_n<int32_t>(n);
  for (size_t i = 0; i < n; ++i) holder.value.user_data[i] = pc.get_userdata(i);
}

void copy_treeid(AttributeHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.tree_id_set = 1;
  holder.value.tree_id = alloc_n<int32_t>(n);
  for (size_t i = 0; i < n; ++i) holder.value.tree_id[i] = pc.get_treeid(i);
}

void copy_rgb(AttributeHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.rgb_set = 1;
  holder.value.red = alloc_n<uint16_t>(n);
  holder.value.green = alloc_n<uint16_t>(n);
  holder.value.blue = alloc_n<uint16_t>(n);
  for (size_t i = 0; i < n; ++i)
  {
    const auto color = pc.get_color(i);
    holder.value.red[i] = color[0];
    holder.value.green[i] = color[1];
    holder.value.blue[i] = color[2];
  }
}

void export_seeds(SeedHolder& holder, const PointCloud& pc)
{
  const size_t n = pc.true_size();
  holder.value.count = n;
  holder.value.x = alloc_n<double>(n);
  holder.value.y = alloc_n<double>(n);
  holder.value.z = alloc_n<double>(n);
  for (size_t i = 0; i < n; ++i)
  {
    holder.value.x[i] = pc.get_x(i);
    holder.value.y[i] = pc.get_y(i);
    holder.value.z[i] = pc.get_z(i);
  }

  holder.value.tree_id_set = pc.has_treeid() ? 1 : 0;
  if (pc.has_treeid())
  {
    holder.value.tree_id = alloc_n<int32_t>(n);
    for (size_t i = 0; i < n; ++i) holder.value.tree_id[i] = pc.get_treeid(i);
  }
  holder.value.foliage_set = pc.has_foliage() ? 1 : 0;
  if (pc.has_foliage())
  {
    holder.value.foliage = alloc_n<int32_t>(n);
    for (size_t i = 0; i < n; ++i) holder.value.foliage[i] = pc.get_foliage(i);
  }
  holder.value.hag_set = pc.has_hag() ? 1 : 0;
  if (pc.has_hag())
  {
    holder.value.hag = alloc_n<double>(n);
    for (size_t i = 0; i < n; ++i) holder.value.hag[i] = pc.get_hag(i);
  }
  holder.value.pwood_set = pc.has_pwood() ? 1 : 0;
  if (pc.has_pwood())
  {
    holder.value.pwood = alloc_n<float>(n);
    for (size_t i = 0; i < n; ++i) holder.value.pwood[i] = static_cast<float>(pc.get_pwood(i));
  }
  holder.value.passage_set = pc.has_passage() ? 1 : 0;
  if (pc.has_passage())
  {
    holder.value.passage = alloc_n<int32_t>(n);
    for (size_t i = 0; i < n; ++i) holder.value.passage[i] = pc.get_passage(i);
  }
}

void export_qsm(QSMHolder& holder, const QSM& graph)
{
  holder.value.id = graph.id;
  if (!graph.name.empty()) holder.value.name = dup_string(graph.name);
  holder.value.crs = dup_string(graph.crs);

  if (!graph.messages.empty())
  {
    holder.value.message_count = graph.messages.size();
    holder.value.messages = alloc_n<char*>(graph.messages.size());
    for (size_t i = 0; i < graph.messages.size(); ++i) holder.value.messages[i] = nullptr;
    for (size_t i = 0; i < graph.messages.size(); ++i) holder.value.messages[i] = dup_string(graph.messages[i]);
  }

  const size_t n = graph.edges().size();
  holder.value.cylinder_count = n;
  holder.value.cylinders = alloc_n<ArborBridgeCylinder>(n);
  size_t i = 0;
  for (const auto& kv : graph.edges())
  {
    const QSMNode& src = graph.node(kv.second.source);
    const QSMNode& tgt = graph.node(kv.second.target);
    const QSMEdge& edge = kv.second.data;
    ArborBridgeCylinder& row = holder.value.cylinders[i++];
    row.start_x = src.x;
    row.start_y = src.y;
    row.start_z = src.z;
    row.end_x = tgt.x;
    row.end_y = tgt.y;
    row.end_z = tgt.z;
    row.cyl_id = kv.second.target;
    row.parent_id = kv.second.source;
    row.axis_id = static_cast<int32_t>(edge.axis_id);
    row.branch_order = edge.branch_order;
    row.quality = edge.quality;
    row.radius = shown(edge.radius, arbor::qsm::RADIUS_UNSET);
    row.dist_to_root = shown(edge.distance_to_root, arbor::qsm::DISTANCE_TO_ROOT_UNSET);
    row.subtree_length = shown(edge.subtree_length, arbor::qsm::SUBTREE_LENGTH_UNSET);
  }
}

void export_qsf(QSFHolder& holder, const QSF& forest)
{
  const auto& models = forest.get_qsm_map();
  holder.value.count = models.size();
  holder.value.models = alloc_n<ArborBridgeQSM>(models.size());
  if (holder.value.models)
    memset(holder.value.models, 0, models.size() * sizeof(ArborBridgeQSM));

  size_t i = 0;
  for (const auto& item : models)
  {
    QSMHolder one;
    export_qsm(one, item.second);
    holder.value.models[i++] = one.release();
  }
}

QSM graph_from(const ArborBridgeQSM& in)
{
  QSM graph;
  graph.id = in.id;
  if (in.name && in.name[0] != '\0') graph.name = in.name;
  else graph.name = "tree_" + std::to_string(graph.id);
  if (in.crs) graph.crs = in.crs;
  if (in.messages)
  {
    graph.messages.reserve(in.message_count);
    for (size_t i = 0; i < in.message_count; ++i)
    {
      if (in.messages[i]) graph.messages.emplace_back(in.messages[i]);
    }
  }

  if (in.cylinder_count > 0 && !in.cylinders)
    throw std::runtime_error("QSM is missing cylinders");

  constexpr int digits = 6;
  const double factor = std::pow(10.0, digits);
  struct CoordKey {
    int x, y, z;
    bool operator==(const CoordKey& other) const noexcept
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct CoordKeyHash {
    size_t operator()(const CoordKey& key) const noexcept
    {
      size_t h1 = std::hash<int>{}(key.x);
      size_t h2 = std::hash<int>{}(key.y);
      size_t h3 = std::hash<int>{}(key.z);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };
  std::unordered_map<CoordKey, int, CoordKeyHash> coord_to_node;
  auto node_id = [&](double x, double y, double z) -> int {
    CoordKey key{
      static_cast<int>(std::llround(x * factor)),
      static_cast<int>(std::llround(y * factor)),
      static_cast<int>(std::llround(z * factor))
    };
    auto found = coord_to_node.find(key);
    if (found != coord_to_node.end()) return found->second;
    int id = graph.add_node(QSMNode{x, y, z});
    coord_to_node.emplace(key, id);
    return id;
  };

  for (size_t i = 0; i < in.cylinder_count; ++i)
  {
    const ArborBridgeCylinder& row = in.cylinders[i];
    const int src = node_id(row.start_x, row.start_y, row.start_z);
    const int tgt = node_id(row.end_x, row.end_y, row.end_z);
    QSMEdge edge;
    edge.id = row.cyl_id;
    edge.source = static_cast<uint32_t>(src);
    edge.target = static_cast<uint32_t>(tgt);
    edge.radius = stored(row.radius, arbor::qsm::RADIUS_UNSET);
    edge.conic_allometry = arbor::qsm::RADIUS_UNSET;
    edge.axis_id = static_cast<uint32_t>(row.axis_id);
    edge.branch_order = static_cast<uint8_t>(row.branch_order);
    edge.distance_to_root = stored(row.dist_to_root, arbor::qsm::DISTANCE_TO_ROOT_UNSET);
    edge.subtree_length = stored(row.subtree_length, arbor::qsm::SUBTREE_LENGTH_UNSET);
    edge.subtree_max_endZ = arbor::qsm::SUBTREE_MAXZ_UNSET;
    edge.subtree_volume = arbor::qsm::SUBTREE_VOLUME_UNSET;
    edge.quality = static_cast<uint8_t>(row.quality);
    graph.add_edge(src, tgt, edge);
  }

  graph.validate();
  return graph;
}

const ArborBridgeCloud& require_cloud(const ArborBridgeCloud *cloud)
{
  if (!cloud) throw std::runtime_error("missing point cloud");
  return *cloud;
}

const ArborBridgeParams& require_params(const ArborBridgeParams *params)
{
  if (!params) throw std::runtime_error("missing parameters");
  return *params;
}

} // namespace

int arbor_bridge_segment_ground(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeAttributes *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!out) throw std::runtime_error("missing output");
    PointCloud pc = load(require_cloud(cloud), OrderSlot::None);
    arbor::segment::segment_ground(pc, make_params(require_params(params)));
    AttributeHolder holder;
    copy_classification(holder, pc);
    copy_hag(holder, pc);
    *out = holder.release();
  });
}

int arbor_bridge_segment_semantic(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeAttributes *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!out) throw std::runtime_error("missing output");
    PointCloud pc = load(require_cloud(cloud), OrderSlot::TreeID);
    PointCloud dtm = arbor::dtm::dtm(pc);
    arbor::segment::segment_semantic(pc, dtm, make_params(require_params(params)));
    AttributeHolder holder;
    set_order(holder, pc, OrderSlot::TreeID);
    copy_foliage(holder, pc);
    copy_passage(holder, pc);
    copy_userdata(holder, pc);
    *out = holder.release();
  });
}

int arbor_bridge_segment_instance(const ArborBridgeCloud *cloud, const ArborBridgeCloud *seeds, const ArborBridgeParams *params, ArborBridgeAttributes *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!out) throw std::runtime_error("missing output");
    if (!seeds) throw std::runtime_error("missing seeds");
    PointCloud pc = load(require_cloud(cloud), OrderSlot::Passage);
    PointCloud seed_cloud = load(*seeds, OrderSlot::None);
    arbor::segment::segment_instance(pc, seed_cloud, make_params(require_params(params)));
    const size_t active = pc.size();
    for (size_t i = 0; i < active; ++i)
    {
      if (pc.get_treeid(i) == -1) pc.set_treeid(i, std::numeric_limits<int32_t>::min());
    }
    AttributeHolder holder;
    set_order(holder, pc, OrderSlot::Passage);
    copy_foliage(holder, pc);
    copy_treeid(holder, pc);
    *out = holder.release();
  });
}

int arbor_bridge_find_seeds(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeSeedCloud *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!out) throw std::runtime_error("missing output");
    PointCloud pc = load(require_cloud(cloud), OrderSlot::None, true);
    PointCloud seeds = arbor::seeds::find_seeds(pc, make_params(require_params(params)));
    SeedHolder holder;
    export_seeds(holder, seeds);
    *out = holder.release();
  });
}

int arbor_bridge_homogeneization(const ArborBridgeCloud *cloud, double res, uint8_t *keep, char **error)
{
  return guard(error, [&] {
    PointCloud pc = load(require_cloud(cloud), OrderSlot::None);
    if (pc.true_size() > 0 && !keep) throw std::runtime_error("missing keep mask");
    std::vector<bool> mask = arbor::utils::homogeneization(pc, res, true);
    if (mask.size() != pc.true_size()) throw std::runtime_error("homogeneization returned the wrong number of points");
    for (size_t i = 0; i < mask.size(); ++i) keep[i] = mask[i] ? 1 : 0;
  });
}

int arbor_bridge_wood_likelihood(const ArborBridgeCloud *cloud, int32_t k, float *pwood, char **error)
{
  return guard(error, [&] {
    PointCloud pc = load(require_cloud(cloud), OrderSlot::None);
    if (pc.true_size() > 0 && !pwood) throw std::runtime_error("missing pwood");
    std::vector<float> scores = arbor::utils::anisotropy(pc, k);
    if (scores.size() != pc.true_size()) throw std::runtime_error("wood likelihood returned the wrong number of points");
    for (size_t i = 0; i < scores.size(); ++i) pwood[i] = scores[i];
  });
}

int arbor_bridge_colorize(const ArborBridgeCloud *cloud, int darken_foliage, ArborBridgeAttributes *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!out) throw std::runtime_error("missing output");
    const ArborBridgeCloud& in = require_cloud(cloud);
    if (!in.tree_id) throw std::runtime_error("No treeID in this point cloud");
    if (!in.red || !in.green || !in.blue) throw std::runtime_error("RGB memory not allocated");
    PointCloud pc = load(in, OrderSlot::None);
    if (pc.has_userdata())
    {
      for (size_t i = 0; i < pc.true_size(); ++i)
      {
        if (pc.get_userdata(i) != 0) pc.set_treeid(i, -pc.get_treeid(i));
      }
    }
    pc.colorize_trees(darken_foliage != 0);
    AttributeHolder holder;
    copy_rgb(holder, pc);
    *out = holder.release();
  });
}

int arbor_bridge_qsm(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeQSM *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!out) throw std::runtime_error("missing output");
    PointCloud pc = load(require_cloud(cloud), OrderSlot::None);
    QSM model = arbor::qsm::qsm(pc, make_params(require_params(params)));
    QSMHolder holder;
    export_qsm(holder, model);
    *out = holder.release();
  });
}

int arbor_bridge_qsf(const ArborBridgeCloud *cloud, double min_height, const ArborBridgeParams *params, ArborBridgeQSF *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!out) throw std::runtime_error("missing output");
    PointCloud pc = load(require_cloud(cloud), OrderSlot::None);
    QSF forest = arbor::qsm::qsf(pc, min_height, make_params(require_params(params)));
    QSFHolder holder;
    export_qsf(holder, forest);
    *out = holder.release();
  });
}

int arbor_bridge_qsm_dbh(const ArborBridgeQSM *model, double breast_height, ArborBridgeDBH *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!model) throw std::runtime_error("missing QSM");
    if (!out) throw std::runtime_error("missing output");
    QSM graph = graph_from(*model);
    double xyz[3] = {};
    double normal[3] = {};
    out->dbh = graph.dbh(breast_height, xyz, normal);
    out->x = xyz[0];
    out->y = xyz[1];
    out->z = xyz[2];
    out->nx = normal[0];
    out->ny = normal[1];
    out->nz = normal[2];
  });
}

int arbor_bridge_qsm_stem(const ArborBridgeQSM *model, ArborBridgeQSM *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!model) throw std::runtime_error("missing QSM");
    if (!out) throw std::runtime_error("missing output");
    QSM graph = graph_from(*model);
    QSM stem = graph.stem();
    if (stem.crs.empty()) stem.crs = graph.crs;
    QSMHolder holder;
    export_qsm(holder, stem);
    *out = holder.release();
  });
}

int arbor_bridge_qsm_merchantable(const ArborBridgeQSM *model, double min_radius, double min_axis_length, ArborBridgeQSM *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!model) throw std::runtime_error("missing QSM");
    if (!out) throw std::runtime_error("missing output");
    QSM graph = graph_from(*model);
    QSM kept = graph.merchantable(min_radius, min_axis_length);
    if (kept.crs.empty()) kept.crs = graph.crs;
    QSMHolder holder;
    export_qsm(holder, kept);
    *out = holder.release();
  });
}

int arbor_bridge_qsm_read(const char *path, ArborBridgeQSM *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!path) throw std::runtime_error("missing path");
    if (!out) throw std::runtime_error("missing output");
    QSM graph;
    graph.read(path);
    QSMHolder holder;
    export_qsm(holder, graph);
    *out = holder.release();
  });
}

int arbor_bridge_qsm_write(const ArborBridgeQSM *model, const char *path, int binary, char **error)
{
  return guard(error, [&] {
    if (!model) throw std::runtime_error("missing QSM");
    if (!path) throw std::runtime_error("missing path");
    QSM graph = graph_from(*model);
    graph.write(path, binary != 0);
  });
}

int arbor_bridge_qsf_read(const char *path, ArborBridgeQSF *out, char **error)
{
  if (out) *out = {};
  return guard(error, [&] {
    if (!path) throw std::runtime_error("missing path");
    if (!out) throw std::runtime_error("missing output");
    QSF forest = QSF::read(path);
    QSFHolder holder;
    export_qsf(holder, forest);
    *out = holder.release();
  });
}

int arbor_bridge_qsf_write(const ArborBridgeQSM *models, size_t count, const char *path, const char *format, int binary, char **error)
{
  return guard(error, [&] {
    if (count > 0 && !models) throw std::runtime_error("missing QSF");
    if (!path) throw std::runtime_error("missing path");
    if (!format) throw std::runtime_error("missing format");
    QSF forest;
    for (size_t i = 0; i < count; ++i) forest.add_qsm(graph_from(models[i]));
    forest.write(path, format, binary != 0);
  });
}
