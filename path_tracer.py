from collections import namedtuple
from dataclasses import dataclass
from datetime import datetime
import functools
import math
from multiprocessing import Pool
import random
import time
from typing import ClassVar, List, NamedTuple, Optional, Tuple, Union


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    up: ClassVar["Vec3"]
    down: ClassVar["Vec3"]
    one: ClassVar["Vec3"]
    zero: ClassVar["Vec3"]

    def __add__(self, other) -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other) -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, value: float) -> "Vec3":
        return Vec3(self.x * value, self.y * value, self.z * value)

    def __rmul__(self, value: float) -> "Vec3":
        return self.__mul__(value)

    def __neg__(self) -> "Vec3":
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(s, o: "Vec3") -> "Vec3":
        return Vec3(s.y * o.z - s.z * o.y, s.z * o.x - s.x * o.z, s.x * o.y - s.y * o.x)

    def normalize(self) -> "Vec3":
        length = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        self.x /= length
        self.y /= length
        self.z /= length
        return self


Vec3.up = Vec3(0.0, 1.0, 0.0)
Vec3.down = Vec3(0.0, -1.0, 0.0)
Vec3.zero = Vec3(0.0, 0.0, 0.0)
Vec3.one = Vec3(1.0, 1.0, 1.0)


@dataclass
class Material:
    emittance: float = 0.0
    reflectance: float = 0.0


@dataclass
class Sphere:
    center: Vec3
    radius: float
    material: Material


@dataclass
class Triangle:
    a: Vec3
    b: Vec3
    c: Vec3
    material: Material = Material()

    def __post_init__(self):
        self.normal = (self.a - self.b).cross(self.a - self.c).normalize()


class IntersectionResult(NamedTuple):
    success: bool
    distance: float
    intersection: Vec3
    normal: Vec3
    item: Optional[Union[Sphere, Triangle]]


NoIntersect = IntersectionResult(
    success=False,
    distance=99_999_999_999_999.0,
    intersection=Vec3.zero,
    normal=Vec3.zero,
    item=None,
)


@dataclass
class Ray:
    origin: Vec3
    dir: Vec3

    def intersect(self, obj) -> IntersectionResult:
        if isinstance(obj, Sphere):
            return self._intersect_sphere(obj)
        elif isinstance(obj, Triangle):
            return self._intersect_triangle(obj)
        else:
            assert (False, "Unsupported intersection type")
            return NoIntersect

    def _intersect_sphere(self, sphere: Sphere) -> IntersectionResult:
        oc = self.origin - sphere.center
        a = self.dir.dot(self.dir)
        b = 2.0 * oc.dot(self.dir)
        c = oc.dot(oc) - sphere.radius * sphere.radius
        discriminant = b * b - 4 * a * c
        if discriminant < 0.0:
            return NoIntersect
        else:
            distance = (-b - math.sqrt(discriminant)) / (2.0 * a)
            if distance < 0.0:
                return NoIntersect
            intersection = self.origin + distance * self.dir
            normal = (intersection - sphere.center).normalize()
            return IntersectionResult(
                success=True,
                distance=distance,
                intersection=intersection,
                normal=normal,
                item=sphere,
            )

    def _intersect_triangle(self, triangle: Triangle) -> IntersectionResult:
        """ Möller–Trumbore intersection """

        epsilon = 0.000_001
        v0, v1, v2 = triangle.a, triangle.b, triangle.c
        edge1 = v1 - v0
        edge2 = v2 - v0
        h = self.dir.cross(edge2)
        a = edge1.dot(h)
        if a > -epsilon and a < epsilon:
            return NoIntersect
        f = 1.0 / a
        s = self.origin - v0
        u = f * s.dot(h)
        if u < 0.0 or u > 1.0:
            return NoIntersect
        q = s.cross(edge1)
        v = f * self.dir.dot(q)
        if v < 0.0 or u + v > 1.0:
            return NoIntersect
        distance = f * edge2.dot(q)
        if distance > epsilon:
            intersection = self.origin + self.dir * distance
            return IntersectionResult(
                success=True,
                distance=distance,
                intersection=intersection,
                normal=triangle.normal,
                item=triangle,
            )
        else:
            return NoIntersect


@dataclass
class Camera:
    position: Vec3
    target: Vec3
    up: Vec3

    resolution: tuple = (100, 100)

    def __post_init__(self):
        self.dir = (self.target - self.position).normalize()
        self.left = self.dir.cross(self.up)
        self.up = self.left.cross(self.dir)

    def generate_ray(self, i: int, j: int) -> Ray:
        ray_origin = self.position
        scale_left = -1.0 + 2.0 * i / self.resolution[0]
        scale_up = -1.0 + 2.0 * j / self.resolution[1]
        ray_dir = self.dir + self.left * scale_left + self.up * scale_up
        return Ray(ray_origin, ray_dir.normalize())


def save_img(filepath: str, img: List[List[int]]):
    with open(filepath, "w") as f:
        f.write("P2\n")
        f.write(f"{len(img[0])} {len(img)}\n")
        f.write("255\n")
        for row in reversed(img):
            for color in row:
                f.write(f"{color} ")
            f.write("\n")


def random_unit_vec_in_hemisphere(normal: Vec3) -> Vec3:
    theta = math.pi * random.uniform(0, 1)
    phi = 2 * math.pi * random.uniform(0, 1)
    sin_theta = math.sin(theta)
    x = sin_theta * math.cos(phi)
    y = sin_theta * math.sin(phi)
    z = math.cos(theta)
    result = Vec3(x, y, z)
    if result.dot(normal) >= 0.0:
        return result
    else:
        return -result


class PathTracer:
    def render(
        self, camera: Camera, scene: Tuple[Triangle, Sphere], samples: int
    ) -> List[List[int]]:
        img = [
            [0 for x in range(camera.resolution[0])]
            for x in range(camera.resolution[1])
        ]

        def args_provider():
            nonlocal samples, camera, scene
            for j in range(camera.resolution[1]):
                for i in range(camera.resolution[0]):
                    ray = camera.generate_ray(i, j)
                    for s in range(samples):
                        yield (ray, 0, scene, None)

        def init_process():
            random.seed()

        process_count = 4
        with Pool(processes=process_count, initializer=init_process) as pool:
            colors = pool.imap(
                self._path_trace_from_tuple,
                args_provider(),
                samples * camera.resolution[0],
            )
            for j in range(camera.resolution[1]):
                for i in range(camera.resolution[0]):
                    for s in range(samples):
                        img[j][i] += next(colors)

        for j in range(camera.resolution[1]):
            for i in range(camera.resolution[0]):
                img[j][i] = max(0, min(int(img[j][i] / samples), 255))

        return img

    def path_trace(
        self,
        ray: Ray,
        depth: int,
        items: Tuple[Triangle, Sphere],
        from_item: Optional[Union[Triangle, Sphere]] = None,
    ) -> int:
        max_depth = 5
        if depth > max_depth:
            return 0

        closest_hit = NoIntersect
        for item in items:
            if from_item is not None and item is from_item:
                continue
            hit = ray.intersect(item)
            if hit.success and closest_hit.distance > hit.distance:
                closest_hit = hit

        if not closest_hit.success:
            return 0

        new_ray_dir = random_unit_vec_in_hemisphere(closest_hit.normal)
        new_ray = Ray(closest_hit.intersection, new_ray_dir)

        cos_theta = new_ray.dir.dot(closest_hit.normal)
        assert closest_hit.item is not None
        reflectance = closest_hit.item.material.reflectance
        emittance = closest_hit.item.material.emittance
        if reflectance < 0.01:
            return int(emittance)
        color_incoming = self.path_trace(new_ray, depth + 1, items, closest_hit.item)
        final_color = emittance + (reflectance * color_incoming * cos_theta * 2.0)
        return int(final_color)

    def _path_trace_from_tuple(
        self,
        args: Tuple[
            Ray, int, Tuple[Triangle, Sphere], Optional[Union[Triangle, Sphere]]
        ],
    ) -> int:
        return self.path_trace(*args)
