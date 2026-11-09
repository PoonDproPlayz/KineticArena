#include "StlMeshSdf.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace {
using Vec3 = std::array<double, 3>;

constexpr const char* kPluginName = "combatsim.sdf.stl_mesh";
constexpr int kLeafTriangleCount = 8;

Vec3 add(Vec3 a, const Vec3& b) {
  for (int axis = 0; axis < 3; ++axis) a[axis] += b[axis];
  return a;
}

Vec3 sub(Vec3 a, const Vec3& b) {
  for (int axis = 0; axis < 3; ++axis) a[axis] -= b[axis];
  return a;
}

Vec3 mul(Vec3 a, double scale) {
  for (double& value : a) value *= scale;
  return a;
}

double dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1],
          a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

double norm(const Vec3& value) { return std::sqrt(dot(value, value)); }

Vec3 normalized(Vec3 value) {
  const double length = norm(value);
  if (length > 1e-15) return mul(value, 1.0 / length);
  return {0.0, 0.0, 1.0};
}

const Vec3& parity_ray_direction() {
  static const Vec3 direction = normalized({1.0, 0.3713906763541037,
                                             0.1924500897298753});
  return direction;
}

struct Triangle {
  Vec3 a{};
  Vec3 b{};
  Vec3 c{};
  Vec3 low{};
  Vec3 high{};
  Vec3 center{};
  Vec3 normal{};
  // The parity ray has a fixed direction, so cache its Moller-Trumbore
  // coefficients once instead of rebuilding cross products per distance query.
  Vec3 ray_u_axis{};
  Vec3 ray_v_axis{};
  Vec3 ray_t_axis{};
  double ray_inverse_determinant = 0.0;
  std::array<int, 3> vertex{};
};

struct BvhNode {
  Vec3 low{};
  Vec3 high{};
  int begin = 0;
  int end = 0;
  int left = -1;
  int right = -1;
};

struct MeshSdf {
  struct CachedQuery {
    Vec3 point{};
    Vec3 gradient{};
    double distance = 0.0;
    bool valid = false;
  };

  std::vector<Triangle> triangles;
  std::vector<int> order;
  std::vector<BvhNode> nodes;
  bool watertight = false;
  bool reliable_normal_sign = false;
  double normal_sign_orientation = 1.0;
  mutable std::mutex query_cache_mutex;
  mutable std::array<CachedQuery, 16> query_cache{};
  mutable size_t next_query_cache_slot = 0;
  mutable int last_closest_triangle = -1;

  int BuildNode(int begin, int end) {
    BvhNode node;
    node.begin = begin;
    node.end = end;
    node.low.fill(std::numeric_limits<double>::infinity());
    node.high.fill(-std::numeric_limits<double>::infinity());
    Vec3 center_low = node.low;
    Vec3 center_high = node.high;
    for (int i = begin; i < end; ++i) {
      const Triangle& triangle = triangles[order[i]];
      for (int axis = 0; axis < 3; ++axis) {
        node.low[axis] = std::min(node.low[axis], triangle.low[axis]);
        node.high[axis] = std::max(node.high[axis], triangle.high[axis]);
        center_low[axis] = std::min(center_low[axis], triangle.center[axis]);
        center_high[axis] = std::max(center_high[axis], triangle.center[axis]);
      }
    }
    const int index = int(nodes.size());
    nodes.push_back(node);
    if (end - begin <= kLeafTriangleCount) return index;

    int axis = 0;
    for (int candidate = 1; candidate < 3; ++candidate)
      if (center_high[candidate] - center_low[candidate] >
          center_high[axis] - center_low[axis]) axis = candidate;
    const int middle = begin + (end - begin) / 2;
    std::nth_element(order.begin() + begin, order.begin() + middle,
                     order.begin() + end, [&](int lhs, int rhs) {
      return triangles[lhs].center[axis] < triangles[rhs].center[axis];
    });
    const int left = BuildNode(begin, middle);
    const int right = BuildNode(middle, end);
    nodes[index].left = left;
    nodes[index].right = right;
    return index;
  }

  static double PointBoxDistanceSquared(const Vec3& point, const BvhNode& node) {
    double result = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double gap = point[axis] < node.low[axis] ? node.low[axis] - point[axis] :
                         point[axis] > node.high[axis] ? point[axis] - node.high[axis] : 0.0;
      result += gap * gap;
    }
    return result;
  }

  static bool RayHitsBox(const Vec3& origin, const Vec3& direction,
                         const BvhNode& node) {
    double near = 1e-10;
    double far = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(direction[axis]) < 1e-15) {
        if (origin[axis] < node.low[axis] || origin[axis] > node.high[axis]) return false;
        continue;
      }
      double first = (node.low[axis] - origin[axis]) / direction[axis];
      double last = (node.high[axis] - origin[axis]) / direction[axis];
      if (first > last) std::swap(first, last);
      near = std::max(near, first);
      far = std::min(far, last);
      if (near > far) return false;
    }
    return far > near;
  }

  static Vec3 ClosestPoint(const Vec3& point, const Triangle& triangle) {
    const Vec3 ab = sub(triangle.b, triangle.a);
    const Vec3 ac = sub(triangle.c, triangle.a);
    const Vec3 ap = sub(point, triangle.a);
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) return triangle.a;

    const Vec3 bp = sub(point, triangle.b);
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) return triangle.b;

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
      return add(triangle.a, mul(ab, d1 / (d1 - d3)));

    const Vec3 cp = sub(point, triangle.c);
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return triangle.c;

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
      return add(triangle.a, mul(ac, d2 / (d2 - d6)));

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
      const Vec3 bc = sub(triangle.c, triangle.b);
      return add(triangle.b, mul(bc, (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }

    const double inverse = 1.0 / (va + vb + vc);
    return add(add(triangle.a, mul(ab, vb * inverse)), mul(ac, vc * inverse));
  }

  static bool RayHitsTriangle(const Vec3& origin, const Triangle& triangle) {
    if (triangle.ray_inverse_determinant == 0.0) return false;
    const double inverse = triangle.ray_inverse_determinant;
    const Vec3 s = sub(origin, triangle.a);
    const double u = inverse * dot(s, triangle.ray_u_axis);
    if (u < 1e-10 || u >= 1.0 - 1e-10) return false;
    const double v = inverse * dot(s, triangle.ray_v_axis);
    if (v < 1e-10 || u + v >= 1.0 - 1e-10) return false;
    return inverse * dot(s, triangle.ray_t_axis) > 1e-10;
  }

  void FindClosest(int node_index, const Vec3& point, double& best_squared,
                   Vec3& best_point, Vec3& best_normal,
                   int& best_triangle) const {
    const BvhNode& node = nodes[node_index];
    if (PointBoxDistanceSquared(point, node) > best_squared) return;
    if (node.left < 0) {
      for (int i = node.begin; i < node.end; ++i) {
        const Triangle& triangle = triangles[order[i]];
        const Vec3 candidate = ClosestPoint(point, triangle);
        const Vec3 delta = sub(point, candidate);
        const double distance_squared = dot(delta, delta);
        if (distance_squared < best_squared) {
          best_squared = distance_squared;
          best_point = candidate;
          best_normal = triangle.normal;
          best_triangle = order[i];
        }
      }
      return;
    }

    const double left_distance = PointBoxDistanceSquared(point, nodes[node.left]);
    const double right_distance = PointBoxDistanceSquared(point, nodes[node.right]);
    if (left_distance <= right_distance) {
      FindClosest(node.left, point, best_squared, best_point, best_normal,
                  best_triangle);
      FindClosest(node.right, point, best_squared, best_point, best_normal,
                  best_triangle);
    } else {
      FindClosest(node.right, point, best_squared, best_point, best_normal,
                  best_triangle);
      FindClosest(node.left, point, best_squared, best_point, best_normal,
                  best_triangle);
    }
  }

  void CountRayHits(int node_index, const Vec3& origin, const Vec3& direction,
                    int& count) const {
    const BvhNode& node = nodes[node_index];
    if (!RayHitsBox(origin, direction, node)) return;
    if (node.left < 0) {
      for (int i = node.begin; i < node.end; ++i)
        if (RayHitsTriangle(origin, triangles[order[i]])) ++count;
      return;
    }
    CountRayHits(node.left, origin, direction, count);
    CountRayHits(node.right, origin, direction, count);
  }

  void Query(const Vec3& point, double& distance, Vec3* gradient) const {
    int seed_triangle = -1;
    {
      std::lock_guard<std::mutex> lock(query_cache_mutex);
      for (const CachedQuery& cached : query_cache) {
        if (cached.valid && cached.point == point) {
          distance = cached.distance;
          if (gradient) *gradient = cached.gradient;
          return;
        }
      }
      seed_triangle = last_closest_triangle;
    }

    double distance_squared = std::numeric_limits<double>::infinity();
    Vec3 closest{}, surface_normal{};
    int closest_triangle = -1;
    // SDF contact optimization asks for nearby points repeatedly. Seed the
    // exact BVH search with the previous nearest triangle; its distance is a
    // valid upper bound, so pruning improves without changing the result.
    if (seed_triangle >= 0 && seed_triangle < int(triangles.size())) {
      const Triangle& seed = triangles[seed_triangle];
      closest = ClosestPoint(point, seed);
      const Vec3 delta = sub(point, closest);
      distance_squared = dot(delta, delta);
      surface_normal = seed.normal;
      closest_triangle = seed_triangle;
    }
    FindClosest(0, point, distance_squared, closest, surface_normal,
                closest_triangle);
    const double unsigned_distance = std::sqrt(std::max(0.0, distance_squared));
    bool inside = false;
    if (reliable_normal_sign) {
      // For a consistently wound, closed Fusion solid, triangle normals point
      // out of the material. The nearest-face result gives the exact signed
      // side without a second BVH ray traversal.
      inside = dot(sub(point, closest), surface_normal) *
        normal_sign_orientation < 0.0;
    } else if (watertight) {
      // The fixed direction avoids repeatedly landing on aligned STL edges.
      int hits = 0;
      CountRayHits(0, point, parity_ray_direction(), hits);
      inside = (hits & 1) != 0;
    } else {
      // Open STL surfaces retain the existing nearest-face sign convention,
      // using the closest-face result above instead of traversing the BVH twice.
      inside = dot(sub(point, closest), surface_normal) < 0.0;
    }
    distance = inside ? -unsigned_distance : unsigned_distance;
    Vec3 computed_gradient{};
    if (unsigned_distance > 1e-10) {
      const double sign = inside ? -1.0 : 1.0;
      computed_gradient = mul(sub(point, closest), sign / unsigned_distance);
    } else {
      computed_gradient = normalized(surface_normal);
    }
    if (gradient) *gradient = computed_gradient;

    {
      std::lock_guard<std::mutex> lock(query_cache_mutex);
      CachedQuery& cached = query_cache[next_query_cache_slot++ % query_cache.size()];
      cached.point = point;
      cached.distance = distance;
      cached.gradient = computed_gradient;
      cached.valid = true;
      last_closest_triangle = closest_triangle;
    }
  }

  bool Load(const mjModel* model, int instance) {
    int mesh_id = -1;
    for (int geom = 0; geom < model->ngeom; ++geom) {
      if (model->geom_plugin[geom] != instance) continue;
      if (model->geom_type[geom] != mjGEOM_SDF) continue;
      mesh_id = model->geom_dataid[geom];
      break;
    }
    if (mesh_id < 0 || mesh_id >= model->nmesh) return false;

    const int vertex_address = model->mesh_vertadr[mesh_id];
    const int vertex_count = model->mesh_vertnum[mesh_id];
    const int face_address = model->mesh_faceadr[mesh_id];
    const int face_count = model->mesh_facenum[mesh_id];
    if (vertex_count < 4 || face_count < 4 || vertex_address < 0 || face_address < 0)
      return false;

    triangles.reserve(face_count);
    struct EdgeUse {
      int count = 0;
      int direction_balance = 0;
    };
    std::unordered_map<uint64_t, EdgeUse> edge_uses;
    edge_uses.reserve(size_t(face_count) * 3);
    auto edge_key = [](int first, int second) {
      const uint32_t low = uint32_t(std::min(first, second));
      const uint32_t high = uint32_t(std::max(first, second));
      return (uint64_t(low) << 32) | high;
    };

    double signed_volume = 0.0;
    double absolute_volume = 0.0;
    auto add_edge_use = [&](int first, int second) {
      EdgeUse& use = edge_uses[edge_key(first, second)];
      ++use.count;
      use.direction_balance += first < second ? 1 : -1;
    };
    for (int face = 0; face < face_count; ++face) {
      Triangle triangle;
      for (int corner = 0; corner < 3; ++corner) {
        const int vertex = model->mesh_face[3 * (face_address + face) + corner];
        if (vertex < 0 || vertex >= vertex_count) return false;
        triangle.vertex[corner] = vertex;
      }
      const int v0 = triangle.vertex[0], v1 = triangle.vertex[1], v2 = triangle.vertex[2];
      auto read_vertex = [&](int vertex) {
        const int address = 3 * (vertex_address + vertex);
        return Vec3{model->mesh_vert[address], model->mesh_vert[address + 1],
                    model->mesh_vert[address + 2]};
      };
      triangle.a = read_vertex(v0);
      triangle.b = read_vertex(v1);
      triangle.c = read_vertex(v2);
      triangle.low.fill(std::numeric_limits<double>::infinity());
      triangle.high.fill(-std::numeric_limits<double>::infinity());
      for (int axis = 0; axis < 3; ++axis) {
        triangle.low[axis] = std::min({triangle.a[axis], triangle.b[axis], triangle.c[axis]});
        triangle.high[axis] = std::max({triangle.a[axis], triangle.b[axis], triangle.c[axis]});
        triangle.center[axis] = (triangle.a[axis] + triangle.b[axis] + triangle.c[axis]) / 3.0;
      }
      triangle.normal = normalized(cross(sub(triangle.b, triangle.a), sub(triangle.c, triangle.a)));
      const double triangle_volume = dot(triangle.a,
        cross(triangle.b, triangle.c)) / 6.0;
      signed_volume += triangle_volume;
      absolute_volume += std::abs(triangle_volume);
      const Vec3& ray_direction = parity_ray_direction();
      const Vec3 edge1 = sub(triangle.b, triangle.a);
      const Vec3 edge2 = sub(triangle.c, triangle.a);
      triangle.ray_u_axis = cross(ray_direction, edge2);
      const double determinant = dot(edge1, triangle.ray_u_axis);
      if (std::abs(determinant) >= 1e-12) {
        triangle.ray_inverse_determinant = 1.0 / determinant;
        triangle.ray_v_axis = cross(edge1, ray_direction);
        triangle.ray_t_axis = cross(edge1, edge2);
      }
      triangles.push_back(triangle);
      add_edge_use(v0, v1);
      add_edge_use(v1, v2);
      add_edge_use(v2, v0);
    }
    watertight = std::all_of(edge_uses.begin(), edge_uses.end(),
      [](const auto& edge) { return edge.second.count == 2; });
    const bool consistently_wound = watertight &&
      std::all_of(edge_uses.begin(), edge_uses.end(),
        [](const auto& edge) { return edge.second.direction_balance == 0; });
    reliable_normal_sign = consistently_wound && absolute_volume > 0.0 &&
      std::abs(signed_volume) > absolute_volume * 1e-10;
    if (reliable_normal_sign && signed_volume < 0.0)
      normal_sign_orientation = -1.0;
    order.resize(triangles.size());
    std::iota(order.begin(), order.end(), 0);
    nodes.reserve(triangles.size() * 2);
    BuildNode(0, int(order.size()));
    return true;
  }
};

Vec3 read_point(const mjtNum point[3]) {
  return {double(point[0]), double(point[1]), double(point[2])};
}

MeshSdf* sdf_data(const mjData* data, int instance) {
  if (!data || instance < 0 || instance >= data->nplugin) return nullptr;
  return reinterpret_cast<MeshSdf*>(data->plugin_data[instance]);
}

int plugin_init(const mjModel* model, mjData* data, int instance) {
  auto* sdf = new MeshSdf();
  if (!sdf->Load(model, instance)) {
    delete sdf;
    return -1;
  }
  data->plugin_data[instance] = reinterpret_cast<uintptr_t>(sdf);
  return 0;
}

void plugin_destroy(mjData* data, int instance) {
  delete sdf_data(data, instance);
  data->plugin_data[instance] = 0;
}

mjtNum plugin_distance(const mjtNum point[3], const mjData* data, int instance) {
  const MeshSdf* sdf = sdf_data(data, instance);
  if (!sdf) return std::numeric_limits<mjtNum>::infinity();
  double distance = 0.0;
  sdf->Query(read_point(point), distance, nullptr);
  return mjtNum(distance);
}

void plugin_gradient(mjtNum gradient[3], const mjtNum point[3],
                     const mjData* data, int instance) {
  const MeshSdf* sdf = sdf_data(data, instance);
  if (!sdf) {
    gradient[0] = 0.0;
    gradient[1] = 0.0;
    gradient[2] = 1.0;
    return;
  }
  double distance = 0.0;
  Vec3 value{};
  sdf->Query(read_point(point), distance, &value);
  for (int axis = 0; axis < 3; ++axis) gradient[axis] = mjtNum(value[axis]);
}

}  // namespace

bool RegisterStlMeshSdfPlugin() {
  static std::once_flag registration;
  static bool registered = false;
  std::call_once(registration, [] {
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = kPluginName;
    plugin.capabilityflags |= mjPLUGIN_SDF;
    plugin.nattribute = 0;
    plugin.attributes = nullptr;
    plugin.nstate = [](const mjModel*, int) { return 0; };
    plugin.init = plugin_init;
    plugin.destroy = plugin_destroy;
    plugin.reset = [](const mjModel*, mjtNum*, void*, int) {};
    plugin.compute = [](const mjModel*, mjData*, int, int) {};
    plugin.sdf_distance = plugin_distance;
    plugin.sdf_gradient = plugin_gradient;
    registered = mjp_registerPlugin(&plugin) >= 0;
  });
  return registered;
}
