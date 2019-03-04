#include <math.h>
#include <iostream>
#include <tuple>
#include <string>
#include <vector>
#include <fstream>
#include <variant>
#include <array>
#include <thread>
#include <mutex>

using std::vector;
using std::string;
using std::tuple;

struct Vec3 {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;

  Vec3()=default;
  constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

  constexpr static Vec3 up() { return Vec3{0.f, 1.f, 0.f}; }
  constexpr static Vec3 down() { return Vec3{0.f, -1.f, 0.f}; }
  constexpr static Vec3 one() { return Vec3{1.f, 1.f, 1.f}; }
  constexpr static Vec3 zero() { return Vec3{0.f, 0.f, 0.f}; }

  Vec3 operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
  Vec3 operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
  Vec3 operator*(float val) const { return Vec3(x * val, y * val, z * val); }
  Vec3 operator-() const { return Vec3(-x, -y, -z); }

  float dot(const Vec3& other) const {
    return x * other.x + y * other.y + z * other.z;
  }

  Vec3 cross(const Vec3& o) const {
    return Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
  }

  Vec3& normalize() {
    float len = sqrt(x * x + y * y + z * z);
    x /= len;
    y /= len;
    z /= len;
    return *this;
  }
};

Vec3 operator*(float val, const Vec3& v) { return Vec3(v.x * val, v.y * val, v.z * val); }

struct Material {
  Material()=default;
  Material(float e, float r) : emittance(e), reflectance(r) {}
  float emittance = 0.f;
  float reflectance = 0.f;
};

inline int next_id() {
  static int id = 0;
  return id++;
}

struct Sphere {
  Vec3 center;
  float radius;
  Material material;
  int id = next_id();
};

struct Triangle {
  Vec3 a, b, c;
  Material material;
  Vec3 normal = Vec3::zero();
  int id = next_id();
  Triangle()=default;
  Triangle(const Vec3& a, const Vec3& b, const Vec3& c) : a(a), b(b), c(c) {
    normal = (a - b).cross(a - c).normalize();
  }
  Triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Material& m)
    : Triangle(a, b, c) {
    material = m;
  } 
};

struct Ray {
  Vec3 origin, dir;

  Ray(const Vec3& origin, const Vec3& dir) : origin(origin), dir(dir) {}

  using HitResult = tuple<bool, float, Vec3, Vec3, Material, int>;
  static constexpr HitResult no_intersect
    = HitResult(false, 9999999999.f, Vec3::zero(), Vec3::zero(), Material(), -1);

  HitResult intersect(const Sphere& s) const {
    auto oc = origin - s.center;
    auto a = dir.dot(dir);
    auto b = 2.f * oc.dot(dir);
    auto c = oc.dot(oc) - s.radius * s.radius;
    auto discriminant = b*b - 4*a*c;
    if (discriminant < 0.f) {
      return Ray::no_intersect;
    } else {
      auto distance = (-b - sqrt(discriminant)) / (2.f * a);
      if (distance < 0.f) {
        return Ray::no_intersect;
      }
      auto intersection = origin + distance * dir;
      auto normal = (intersection - s.center).normalize();
      return HitResult(true, distance, intersection, normal, s.material, s.id);
    }
  }

  HitResult intersect(const Triangle& t) const {
    auto epsilon = 0.000001f;
    auto v0 = t.a;
    auto v1 = t.b;
    auto v2 = t.c;
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;
    auto h = dir.cross(edge2);
    auto a = edge1.dot(h);
    if ((a > -epsilon) && (a < epsilon)) {
      return Ray::no_intersect;
    }
    auto f = 1.f / a;
    auto s = origin - v0;
    auto u = f * s.dot(h);
    if ((u < 0.f) || (u > 1.f)) {
      return Ray::no_intersect;
    }
    auto q = s.cross(edge1);
    auto v = f * dir.dot(q);
    if ((v < 0.f) || (u + v > 1.f)) {
      return Ray::no_intersect;
    }
    auto distance = f * edge2.dot(q);
    if (distance > epsilon) {
      auto intersect = origin + dir * distance;
      return HitResult(true, distance, intersect, t.normal, t.material, t.id);
    } else {
      return Ray::no_intersect;
    }
  }
};

struct Camera {
  Vec3 position, target, up;
  Vec3 dir, left;
  std::array<int, 2> resolution = { 100, 100 };

  Camera(const Vec3& position, const Vec3& target, const Vec3& up)
    : position(position), target(target), up(up) {
    this->dir = (target - position).normalize();
    this->left = dir.cross(up);
    this->up = left.cross(dir);
  }

  Ray generate_ray(int i, int j) const {
    auto ray_origin = position;
    auto scale_left = -1.f + 2.f * static_cast<float>(i) / resolution[0];
    auto scale_up = -1.f + 2.f * static_cast<float>(j) / resolution[1];
    auto ray_dir = dir + left * scale_left + up * scale_up;
    return Ray(ray_origin, ray_dir.normalize());
  }
};

inline void save_img(const string& filepath, const vector<vector<int>>& data) {
  std::ofstream fout;
  fout.open(filepath);
  fout << "P2\n";
  fout << data.size() << " " << data[0].size() << "\n";
  fout << "255\n";
  for (auto& row : data) {
    for (auto& color : row) {
      fout << color << " ";
    }
    fout << "\n";
  }
  fout.close();
}

inline Vec3 random_unit_vec_in_hemisphere(const Vec3& normal) {
  auto theta = static_cast<float>(M_PI) * std::rand() / (RAND_MAX + 1.f);
  auto phi = 2.f * static_cast<float>(M_PI) * std::rand() / (RAND_MAX + 1.f);
  auto x = sin(theta) * cos(phi);
  auto y = sin(theta) * sin(phi);
  auto z = cos(theta);
  auto result = Vec3(x, y, z);
  if (result.dot(normal) >= 0.f)
    return result;
  else
    return -result;
}

using Object = std::variant<Sphere, Triangle>;

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

inline int path_trace(const Ray& ray,
                      int depth,
                      const vector<Object>& scene,
                      int from_id = -1) {
  const int max_depth = 5;
  if (depth >= max_depth) {
    return 0;
  }

  auto closest_hit = Ray::no_intersect;
  for (auto&& object : scene) {
    std::visit(overloaded {
      [&ray, from_id, &closest_hit](const Sphere& s) {
        if (from_id != s.id) {
          auto hit = ray.intersect(s);
          if (std::get<0>(hit) && (std::get<1>(closest_hit) > std::get<1>(hit))) {
            closest_hit = hit;
          }
        }
      },
      [&ray, from_id, &closest_hit](const Triangle& t) {
        if (from_id != t.id) {
          auto hit = ray.intersect(t);
          if (std::get<0>(hit) && (std::get<1>(closest_hit) > std::get<1>(hit))) {
            closest_hit = hit;
          }
        }
      },
    }, object);
  }

  auto [success, distance, intersection, normal, material, id] = closest_hit;
  if (!success)
    return 0;

  auto new_ray_dir = random_unit_vec_in_hemisphere(normal);
  auto new_ray = Ray(intersection, new_ray_dir);
  auto cos_theta = new_ray.dir.dot(normal);
  auto reflectance = material.reflectance;
  auto emittance = material.emittance;
  if (reflectance < 0.01f) {
    return emittance;
  }
  auto color_incoming = path_trace(new_ray, depth + 1, scene, id);
  auto final_color = emittance + (reflectance * color_incoming * cos_theta * 2.f);
  return final_color;
}

void job(const Camera& camera,
         const vector<Object>& scene,
         int row,
         int samples,
         std::mutex& lock,
         vector<vector<int>>& img) {
  for (int i = 0; i < camera.resolution[0]; ++i) {
    auto ray = camera.generate_ray(i, row);
    for (int s = 0; s < samples; ++s) {
      auto color = path_trace(ray, 0, scene);
      lock.lock();
      img[row][i] += color;
      lock.unlock();
    }
  }
}

int main() {
  auto camera = Camera(Vec3(0, 2, 8.5f),
                       Vec3(0, 0, 0),
                       Vec3::up());
  camera.resolution = { 600, 600 };
  vector<Object> scene = {
    Sphere{ Vec3(-4, 0, 0), 2.0, Material(0.f, 1.f / M_PI) },
    Sphere{ Vec3(0, -1.0, 3), 1.0, Material(0, 1.f / M_PI) },
    Sphere{ Vec3(4.0, 1, 0.0), 3.0, Material(0, 1.f / M_PI) },
    Sphere{ Vec3(0, 5, 0), 0.75, Material(19550, 0.0) },
    Triangle{ Vec3(-10,-2,-10), Vec3(10,-2,10), Vec3(10,-2,-10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,-2,-10), Vec3(-10,-2,10), Vec3(10,-2,10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,6,-10), Vec3(10,6,-10), Vec3(10,6,10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,6,-10), Vec3(10,6,10), Vec3(-10,6,10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,-2,-10), Vec3(10,-2,-10), Vec3(10,6,-10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,-2,-10), Vec3(10,6,-10), Vec3(-10,6,-10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,-2,-10), Vec3(-10,6,10), Vec3(-10,-2,10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,-2,-10), Vec3(-10,6,-10), Vec3(-10,6,10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(10,-2,-10), Vec3(10,-2,10), Vec3(10,6,10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(10,-2,-10), Vec3(10,6,10), Vec3(10,6,-10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,-2,10), Vec3(10,6,10), Vec3(10,-2,10), Material(0,1.f / M_PI) },
    Triangle{ Vec3(-10,-2,10), Vec3(-10,6,10), Vec3(10,6,10), Material(0,1.f / M_PI) },
  };

  vector<vector<int>> img(camera.resolution[0], vector<int>(camera.resolution[1]));

  int samples = 512;
  std::mutex lock;
  vector<std::thread> threads;
  for (int j = 0; j < camera.resolution[1]; ++j) {
    auto t = std::thread(job,
                         std::cref(camera),
                         std::cref(scene),
                         j,
                         samples,
                         std::ref(lock),
                         std::ref(img));
    threads.push_back(std::move(t));
  }

  for (auto&& t : threads) {
    t.join();
  }

  for (int j = 0; j < camera.resolution[1]; ++j) {
    for (int i = 0; i < camera.resolution[0]; ++i) {
      img[j][i] = std::max<int>(0, std::min<int>(img[j][i] / samples, 255));
    }
  }

  save_img("result_cpp.ppm", img);
}

