import rclpy
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import Float64
import math
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from par_interfaces.msg import GamePiece, GamePieces, IVector2
from par_interfaces.srv import WorldToBoard
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

TTL_PER_DETECTION = 3
FULL_BOARD_WIDTH = 9

class GamePieceContainer():

    def __init__(self, piece: GamePiece, ttl: int):
        self.piece = piece
        self.ttl = ttl

    def update(self):
        self.ttl -= 1


class CubeDetectionNode(Node):
    def __init__(self, service_callback_group, depth_callback_group):
        super().__init__('cube_detection_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'detected_cubes_markers', 10)
        self.table_image_subscriber = self.create_subscription(Image, '/camera/depth/table_image_raw', self.depth_image_callback, 10, callback_group=depth_callback_group)
        self.camera_info_subscription = self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.camera_info_callback, 10)
        self.bridge = CvBridge()
        self.camera_model = None
        self.table_height_subscriber = self.create_subscription(Float64,'par/table_height', self.table_height_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.table_height = 0.0
        self.camera_height_subscriber = self.create_subscription(Float64,'par/camera_height', self.camera_height_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.camera_height = 0.0

        self.pieces_publisher = self.create_publisher(
            GamePieces,
            "/par/game_pieces",
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.detected_pieces: dict[int, GamePieceContainer] = {}

        self.world_to_board_client = self.create_client(
            WorldToBoard,
            "/par/world_to_board",
            callback_group=service_callback_group
        )

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)




    
    def uv_to_angle(self, uv):
        if not self.camera_model is None:
            return np.array(self.camera_model.projectPixelTo3dRay(uv))
        else:
            self.get_logger().error("Tried to use camera model before getting camera info!")
            return None

    def camera_info_callback(self, msg):
        if self.camera_model is None:
            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(msg)
            self.get_logger().info("Got the camera info!")

    def depth_image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg)
            processed_image, markers, pieces = self.detect_cubes(depth_image)
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.publisher_.publish(processed_msg)
            self.marker_publisher_.publish(markers)
            self.pieces_publisher.publish(pieces)
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")


    def table_height_callback(self, msg: Float64):
        self.table_height = msg.data
    
    def camera_height_callback(self, msg: Float64):
        self.camera_height = msg.data


    def get_cube(self, depth_image, approx):
        
        center_x = 0.0
        center_y = 0.0
        for p in approx:
            point = p[0]
            center_x += point[0]
            center_y += point[1]
        center_x /= len(approx)
        center_y /= len(approx)

        center_depth = depth_image[math.floor(center_y), math.floor(center_x)]

        return center_x, center_y, center_depth
    
    def get_transform(self, target_frame, source_frame):
        try:
            return self.tf_buffer.lookup_transform(target_frame,
                    source_frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().error('Unable to find the transformation')
            self.get_logger().error(str(e))
            return None

    def transform_cube(self, cube: Point, time):
        transformation = self.get_transform("world", "camera_depth_optical_frame")
        if (transformation == None):
            self.get_logger().error("Failed to get transform!")

        cube_pose = PoseStamped()
        cube_pose.pose.position = cube
        cube_pose.header.frame_id = "camera_depth_optical_frame"
        cube_pose.header.stamp = time
        
        world_pose = do_transform_pose_stamped(cube_pose, transformation).pose

        world_pose.position.z = self.table_height

        return world_pose

    def detect_cubes(self, depth_image):

        # Normalize depth image to 8-bit
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = np.uint8(depth_normalized)

        
        blurred = cv2.GaussianBlur(depth_normalized, (11, 11), 1.0)

        depth_colored = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)

        edges = cv2.Canny(blurred, 20, 80)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        

        for contour in contours:
            area = cv2.contourArea(contour)

            epsilon = 0.05 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if area < 25:
                cv2.drawContours(depth_colored, [approx], -1, (0, 0, 255), 2)
                continue
            if area > 3000:
                cv2.drawContours(depth_colored, [approx], -1, (255, 0, 255), 2)
            
            

            

            if len(approx) >= 4 and cv2.isContourConvex(approx):
                depth_values = depth_image[contour[:, 0, 1], contour[:, 0, 0]]
                mean_depth = np.mean(depth_values)
                if np.isnan(mean_depth) or mean_depth <= 0 or mean_depth > 60000:
                    continue


                center_x, center_y, center_depth = self.get_cube(depth_image, approx)

                if center_depth <= 0 or center_depth > 1000:
                    continue

                color = (255, 0, 0)
                cv2.drawContours(depth_colored, [approx], -1, color, 2)

                # Convert (center_x, center_y) to 3D coordinates
                uv = np.array([center_x, center_y], dtype=np.float32)
                ray = self.uv_to_angle(uv)
                if not ray is None:
                    point_3d = ray*(self.camera_height+self.table_height)

                    # # Check for duplicate markers
                    # if not any(np.allclose(point_3d, existing_cube) for existing_cube in self.detected_cubes):
                    #     self.detected_cubes.append(point_3d)

                    cube_point = Point()
                    cube_point.x = point_3d[0]
                    cube_point.y = point_3d[1]
                    cube_point.z = point_3d[2]

                    cube_pose = self.transform_cube(cube_point, rclpy.time.Time().to_msg())

                    board_pos = self.world_to_board_pos(cube_pose.position)
                    piece = GamePiece()
                    piece.board_position = board_pos
                    piece.world_position = cube_pose.position
                    
                    board_loc_id = self.get_board_loc_id(board_pos.x, board_pos.y)

                    container = GamePieceContainer(piece, ttl=TTL_PER_DETECTION)

                    if (board_loc_id in self.detected_pieces.keys()):
                        container.ttl += TTL_PER_DETECTION
                    self.detected_pieces[board_loc_id] = container
        pieces = GamePieces()
        pieces_array = []
        markers = MarkerArray()
        for key in self.detected_pieces.keys():
            self.detected_pieces[key].update()
            piece_container = self.detected_pieces[key]
            if (piece_container.ttl >= TTL_PER_DETECTION):
                pieces_array.append(piece_container.piece)

                marker = Marker()
                marker.header.frame_id = "world" 
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.CUBE
                marker.id = key
                marker.pose = Pose()
                marker.pose.position = piece_container.piece.world_position
                marker.scale.x = 0.015
                marker.scale.y = 0.015
                marker.scale.z = 0.015
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                markers.markers.append(marker)
        
        pieces.pieces = pieces_array

        return depth_colored, markers, pieces
    
    def world_to_board_pos(self, point: Point) -> IVector2:
        request = WorldToBoard.Request()
        request.world_point = point
        response = self.world_to_board_client.call(request)
        return response.board_pos
    
    def get_board_loc_id(self, x:int, y:int) -> int:
        return FULL_BOARD_WIDTH*y + x

def main(args=None):
    rclpy.init(args=args)


    service_callback_group = MutuallyExclusiveCallbackGroup()

    node = CubeDetectionNode(
        service_callback_group=service_callback_group,
        depth_callback_group=None
    )

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

