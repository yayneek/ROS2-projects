import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import Pose2D
import math
import numpy as np
import matplotlib.pyplot as plt


class Lidar_to_Pose(Node):
    def __init__(self):
        super().__init__('Lidar_to_Pose')

        sub_x = Subscriber(self, LaserScan, 'scan_x')
        sub_y = Subscriber(self, LaserScan, 'scan_y')

        ats = ApproximateTimeSynchronizer([sub_x, sub_y], queue_size=10, slop=0.1)
        ats.registerCallback(self.synced_callback)


        
        self.publisher = self.create_publisher(
            Pose2D,
            '/position_from_lidar',
            10

        )


    def synced_callback(self, scan_x: LaserScan, scan_y: LaserScan):
        points_x = self.scan_to_points(scan_x, [0, 1, 0.1], math.pi*3/2)
        points_y = self.scan_to_points(scan_y, [0.75, 0.5, 0.1], math.pi)
        points = np.vstack([points_x, points_y])

        obb = self.obb_pca(points)
        self.center = obb['center']
        self.angle = obb['angle']

        point_to_publish = Pose2D()
        point_to_publish.x = self.center[0]
        point_to_publish.y = self.center[1]
        point_to_publish.theta = self.angle

        self.publisher.publish(point_to_publish)

        # czyszczenie wykresu i rysowanie nowych punktów
        # self.ax.clear()
        # self.ax.scatter(*zip(*points_x), s=2, c='r', label="Lidar X")
        # self.ax.scatter(*zip(*points_y), s=2, c='b', label="Lidar Y")
        # self.ax.legend()
        # self.ax.set_aspect('equal')
        # self.ax.set_title("Fuzja dwóch lidarów")
        # plt.draw()
        # plt.pause(0.01)  # odświeżenie wykresu
    
    def scan_to_points(self, scan: LaserScan, scanner_position, scanner_rotation):
        def rot_mat(angle):
            return np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle),  np.cos(angle)]
            ])
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                local = np.array([r*math.cos(angle), r*math.sin(angle)])
                global_pos = np.array(scanner_position[:2]) + np.dot(rot_mat(scanner_rotation), local)
                points.append(global_pos)
            angle += scan.angle_increment
        return points

    def obb_pca(self, points):
        pts = np.asarray(points)
        assert pts.ndim == 2 and pts.shape[1] == 2 and pts.shape[0] >= 2

        # centroid i centrowanie
        c = pts.mean(axis=0)
        X = pts - c

        # macierz kowariancji i PCA
        cov = np.cov(X.T)         # 2x2
        eigvals, eigvecs = np.linalg.eigh(cov)  # zapewnia rosnące wartości własne
        # ustawiamy największy wektor własny jako pierwszą kolumnę
        order = eigvals.argsort()[::-1]
        eigvecs = eigvecs[:, order]
        eigvals = eigvals[order]

        # projekcja punktów na osie PCA
        proj = X.dot(eigvecs)     # (N,2) - współrzędne w bazie PCA

        # min/max w układzie PCA -> rozmiary pudełka
        min_proj = proj.min(axis=0)
        max_proj = proj.max(axis=0)
        width = max_proj[0] - min_proj[0]
        height = max_proj[1] - min_proj[1]

        # rogi w układzie PCA (CCW)
        corners_pca = np.array([
            [min_proj[0], min_proj[1]],
            [min_proj[0], max_proj[1]],
            [max_proj[0], max_proj[1]],
            [max_proj[0], min_proj[1]],
        ])

        # przekształć rogi z powrotem do układu globalnego
        corners_global = corners_pca.dot(eigvecs.T) + c

        # kąt orientacji (wektor własny 0 to oś główna)
        main_axis = eigvecs[:, 0]
        angle = math.atan2(main_axis[1], main_axis[0])  # radiany

        return {
            'center': c,
            'width': width,
            'height': height,
            'angle': angle,
            'corners': corners_global  # 4x2
        }


def main(args = None):
    rclpy.init(args=args)
    node = Lidar_to_Pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()