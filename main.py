import math
import numpy as np
import time

class CartesianPoint2D:
    def __init__(self, x: float, y: float):
        self.__x = x
        self.__y = y

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @staticmethod
    def from_polar(polar_point: "PolarPoint") -> "CartesianPoint2D":
        return CartesianPoint2D(polar_point.radius * math.cos(polar_point.angle),
                                polar_point.radius * math.sin(polar_point.angle))

    @staticmethod
    def distance(first_point: "CartesianPoint2D", second_point: "CartesianPoint2D") -> float:
        return math.sqrt(pow((second_point.x - first_point.x),2)
                         + pow((second_point.y - first_point.y),2))

class PolarPoint:
    def __init__(self, radius: float, angle: float):
        self.__radius = radius
        self.__angle = angle

    @property
    def radius(self):
        return self.__radius

    @property
    def angle(self):
        return self.__angle

    @staticmethod
    def from_cartesian(cartesian_point: "CartesianPoint2D") -> "PolarPoint":
        return PolarPoint(math.sqrt(cartesian_point.x*cartesian_point.x + cartesian_point.y * cartesian_point.y),
                          math.atan2(cartesian_point.y, cartesian_point.x))

    @staticmethod
    def distance(first_point: "PolarPoint", second_point: "PolarPoint") -> float:
        return math.sqrt(pow(second_point.radius, 2)
                         + pow(first_point.radius, 2)
                         - 2 * first_point.radius * second_point.radius * math.cos(second_point.angle - first_point.angle))

class CartesianPoint3D:
    def __init__(self, x: float, y: float, z: float):
        self.__x = x
        self.__y = y
        self.__z = z

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    @staticmethod
    def from_spherical_point(spherical_point: "SphericalPoint") -> "CartesianPoint3D":
        return CartesianPoint3D(
            spherical_point.radius * math.sin(spherical_point.polar_angle) * math.cos(spherical_point.azimuth),
            spherical_point.radius * math.sin(spherical_point.polar_angle) * math.sin(spherical_point.azimuth),
            spherical_point.radius * math.cos(spherical_point.polar_angle)
        )

    @staticmethod
    def distance(first_point, second_point) -> float:
        return math.sqrt(pow((second_point.x - first_point.x), 2)
                         + pow((second_point.y - first_point.y), 2)
                         + pow((second_point.z - first_point.z), 2))

class SphericalPoint:
    def __init__(self, radius: float, azimuth: float, polar_angle: float):
        self.__radius = radius
        self.__azimuth = azimuth
        self.__polar_angle = polar_angle

    @property
    def radius(self):
        return self.__radius

    @property
    def azimuth(self):
        return self.__azimuth

    @property
    def polar_angle(self):
        return self.__polar_angle

    @staticmethod
    def from_cartesian_point(cartesian_point: "CartesianPoint3D") -> "SphericalPoint":
        cache_radius = math.sqrt(cartesian_point.x * cartesian_point.x
                                 + cartesian_point.y * cartesian_point.y
                                 + cartesian_point.z * cartesian_point.z)
        return SphericalPoint(
            cache_radius,
            math.atan2(cartesian_point.y, cartesian_point.x),
            math.atan2(cartesian_point.z, cache_radius)
        )

    @staticmethod
    def direct_distance(first_point: "SphericalPoint", second_point: "SphericalPoint") -> float:
        return(
            math.sqrt(abs(
                pow(first_point.radius, 2) + pow(second_point.radius, 2)
                - 2 * first_point.radius * second_point.radius * math.cos(first_point.polar_angle - second_point.polar_angle)
                + math.cos(first_point.azimuth) * math.cos(second_point.azimuth)
            ))
        )

    @staticmethod
    def circular_distance(first_point: "SphericalPoint", second_point: "SphericalPoint") -> float | None:
        if first_point.radius != second_point.radius:
            print("Radius of Both point must be equal! \n")
            return None

        return(
            first_point.radius * math.acos(
            math.sin(first_point.polar_angle) * math.sin(second_point.polar_angle) *
            math.cos(first_point.azimuth - second_point.azimuth) +
            math.cos(first_point.polar_angle) * math.cos(second_point.polar_angle)
        )
        )

def first_task_tests():
    ca2 = CartesianPoint2D(1, 2)
    pp = PolarPoint(1, 0.1)
    ca3 = CartesianPoint3D(2, 5, 4)
    sp = SphericalPoint(4, 0.22, 0.99)
    ca2pp = CartesianPoint2D.from_polar(pp)
    ppca2 = PolarPoint.from_cartesian(ca2)
    ca3sp = CartesianPoint3D.from_spherical_point(sp)
    spca3 = SphericalPoint.from_cartesian_point(ca3)
    print('from polar( radius: ', pp.radius, 'angle: ', pp.angle, ') to cartesian x:', ca2pp.x, ' y: ', ca2pp.y,
          ' and back radius: ', PolarPoint.from_cartesian(ca2pp).radius, 'angle:',
          PolarPoint.from_cartesian(ca2pp).angle)
    print('from cartesian( x:', ca2.x, ' y: ', ca2.y, ') to polar radius: ', ppca2.radius, 'angle:', ppca2.angle,
          ' and back x: ', CartesianPoint2D.from_polar(ppca2).x, ' y: ', CartesianPoint2D.from_polar(ppca2).y)
    print('from spherical( radius: ', sp.radius, ' azimuth:', sp.azimuth, ' polar: ', sp.polar_angle,
          ') to cartesian3d x:', ca3sp.x,
          ' y: ', ca3sp.y, 'z:', ca3sp.z,
          '\n and back radius:', SphericalPoint.from_cartesian_point(ca3sp).radius, ' azimuth:',
          SphericalPoint.from_cartesian_point(ca3sp).azimuth, ' polar: ',
          SphericalPoint.from_cartesian_point(ca3sp).polar_angle, )
    print('from cartesian3d( x: ', ca3.x, ' y: ', ca3.y, 'z:', ca3.z, ') to spherical radius: ',
          spca3.radius, ' azimuth: ', spca3.azimuth, ' radius: ', spca3.polar_angle,
          '\n and back x:', CartesianPoint3D.from_spherical_point(spca3).x,
          ' y: ', CartesianPoint3D.from_spherical_point(spca3).y, 'z:', CartesianPoint3D.from_spherical_point(spca3).z)

def second_task():
    ca2_1 = CartesianPoint2D(1, 5)
    ca2_2 = CartesianPoint2D(0, -9)
    pp_1 = PolarPoint(1, 5)
    pp_2 = PolarPoint(0, -9)
    ca3_1 = CartesianPoint3D(3, 0, 9)
    ca3_2 = CartesianPoint3D(-2, 10, 6)
    sp_1 = SphericalPoint(12, 1, 1.15)
    sp_2 = SphericalPoint(6, -0.1, 0.5)
    sp_3 = SphericalPoint(12, 0.15, 0.89)
    print('\n', 'CartesianPoint2D', '\n')
    print('first point: x:', ca2_1.x,' y: ', ca2_1.y,
          ' \nsecond point: x:', ca2_2.x,' y: ', ca2_2.y,
          ' \ndistance: ', CartesianPoint2D.distance(ca2_1, ca2_2))
    print('\n', 'PolarPoint', '\n')
    print('first point: radius:', pp_1.radius,' angle: ', pp_1.angle,
          ' \nsecond point: radius:', pp_2.radius,' angle: ', pp_2.angle,
          ' \ndistance: ', PolarPoint.distance(pp_1, pp_2))
    print('\n', 'CartesianPoint3D', '\n')
    print('first point: x:', ca3_1.x, ' y: ', ca3_1.y, ' z: ', ca3_1.z,
          ' \nsecond point: x:', ca3_2.x, ' y: ', ca3_2.y, ' z: ', ca3_2.z,
          ' \ndistance: ', CartesianPoint3D.distance(ca3_1, ca3_2))
    print('\n', 'SphericalPoint Direct', '\n')
    print('first point: radius:', sp_1.radius, ' azimuth: ', sp_1.azimuth, ' polar_angle: ', sp_1.polar_angle,
          ' \nsecond point: radius:', sp_2.radius, ' azimuth: ', sp_2.azimuth, ' polar_angle: ', sp_2.polar_angle,
          ' \ndistance: ', SphericalPoint.direct_distance(sp_1, sp_2))
    print('\n', 'SphericalPoint Circular', '\n')
    print('first point: radius:', sp_1.radius, ' azimuth: ', sp_1.azimuth, ' polar_angle: ', sp_1.polar_angle,
          ' \nsecond point: radius:', sp_2.radius, ' azimuth: ', sp_2.azimuth, ' polar_angle: ', sp_2.polar_angle,
          ' \ndistance: ', SphericalPoint.circular_distance(sp_1, sp_2))
    print('\n', 'SphericalPoint Circular', '\n')
    print('first point: radius:', sp_1.radius, ' azimuth: ', sp_1.azimuth, ' polar_angle: ', sp_1.polar_angle,
          ' \nsecond point: radius:', sp_3.radius, ' azimuth: ', sp_3.azimuth, ' polar_angle: ', sp_3.polar_angle,
          ' \ndistance: ', SphericalPoint.circular_distance(sp_1, sp_3))

#Data ranges
RADIUS_RANGE = (-100, 100)
POLAR_RANGE = (-3.14, 3.14)
AZIMUTH_RANGE = (-3.14, 3.14)
COUPLE_COUNT = 100000

if __name__ == '__main__':
    print('\n', '________________First Task_________________', '\n')
    first_task_tests()
    print('\n', '________________Second Task_________________', '\n')
    second_task()
    print('\n', '________________Third Task_________________', '\n')
    # Prepare and generate
    radius_values_1 = np.random.uniform(low=RADIUS_RANGE[0], high=RADIUS_RANGE[1], size=COUPLE_COUNT)
    radius_values_2 = np.random.uniform(low=RADIUS_RANGE[0], high=RADIUS_RANGE[1], size=COUPLE_COUNT)
    polar_values_1 = np.random.uniform(low=POLAR_RANGE[0], high=POLAR_RANGE[1], size=COUPLE_COUNT)
    polar_values_2 = np.random.uniform(low=POLAR_RANGE[0], high=POLAR_RANGE[1], size=COUPLE_COUNT)
    radius_values_3 = np.random.uniform(low=RADIUS_RANGE[0], high=RADIUS_RANGE[1], size=COUPLE_COUNT)
    radius_values_4 = np.random.uniform(low=RADIUS_RANGE[0], high=RADIUS_RANGE[1], size=COUPLE_COUNT)
    azimuth_values_3 = np.random.uniform(low=AZIMUTH_RANGE[0], high=AZIMUTH_RANGE[1], size=COUPLE_COUNT)
    azimuth_values_4 = np.random.uniform(low=AZIMUTH_RANGE[0], high=AZIMUTH_RANGE[1], size=COUPLE_COUNT)
    polar_values_3 = np.random.uniform(low=POLAR_RANGE[0], high=POLAR_RANGE[1], size=COUPLE_COUNT)
    polar_values_4 = np.random.uniform(low=POLAR_RANGE[0], high=POLAR_RANGE[1], size=COUPLE_COUNT)

    pp_array = [(PolarPoint(radius1, polar1), PolarPoint(radius2, polar2))
         for radius1, polar1, radius2, polar2 in zip(radius_values_1, polar_values_1, radius_values_2, polar_values_2)]
    ca2_array = [(CartesianPoint2D.from_polar(pp_member[0]),CartesianPoint2D.from_polar(pp_member[1])) for pp_member in pp_array]
    sp_array = [(SphericalPoint(radius1, azimuth1, polar1), SphericalPoint(radius1, azimuth2, polar2))
                for radius1, azimuth1, polar1, radius2, azimuth2, polar2 in zip(radius_values_3, azimuth_values_3, polar_values_3, radius_values_4, azimuth_values_4, polar_values_4)]
    ca3_array = [(CartesianPoint3D.from_spherical_point(sp_member[0]),CartesianPoint3D.from_spherical_point(sp_member[1])) for sp_member in sp_array]

    print('\n Test start')

    start = time.time()
    for pp_member in pp_array:
        cache = PolarPoint.distance(pp_member[0],pp_member[1])
    result31a = time.time()-start

    start = time.time()
    for ca2_member in ca2_array:
        cache = CartesianPoint2D.distance(ca2_member[0], ca2_member[1])
    result31b = time.time() - start

    start = time.time()
    for sp_member in sp_array:
        cache = SphericalPoint.direct_distance(sp_member[0],sp_member[1])
    result32a = time.time()-start

    start = time.time()
    for sp_member in sp_array:
        cache = SphericalPoint.circular_distance(sp_member[0], sp_member[1])
    result32b = time.time() - start

    start = time.time()
    for ca3_member in ca3_array:
        cache = CartesianPoint3D.distance(ca3_member[0], ca3_member[1])
    result32c = time.time() - start

    print(
        '\n Result of benchmark:',
        '\n 3.1.A: ',result31a,' , 3.1.B: ', result31b,
        '\n 3.2.A: ', result32a, ' , 3.2.B: ', result32b, ' , 3.2.C: ', result32c
    )