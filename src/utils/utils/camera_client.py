import socketio
import pickle
import cv2
import time
import argparse
import numpy as np

def pixel_to_camera_frame(x: float, y: float, depth_image: np.ndarray, camera_intrinsics: dict) -> tuple[float, float, float]:
    """
    Convert a pixel location (x, y) to a 3D point (x, y, z) in the camera frame.

    Args:
        x (float): Pixel x-coordinate.
        y (float): Pixel y-coordinate.
        depth_image (numpy.ndarray): Depth image where each pixel contains the depth value in millimeters.
        camera_intrinsics (dict): Camera intrinsics containing 'fx', 'fy', 'px', 'py'.

    Returns:
        tuple: A tuple (x, y, z) representing the 3D point in the camera frame.
    """
    fx = camera_intrinsics['fx']
    fy = camera_intrinsics['fy']
    px = camera_intrinsics['px']
    py = camera_intrinsics['py']

    # Get the depth value at the given pixel
    # Bilinear interpolation of depth at (y, x)
    x0, x1 = int(np.floor(x)), int(np.ceil(x))
    y0, y1 = int(np.floor(y)), int(np.ceil(y))

    # Clip indices to image boundaries
    # h, w = depth_image.shape
    # x0 = np.clip(x0, 0, w - 1)
    # x1 = np.clip(x1, 0, w - 1)
    # y0 = np.clip(y0, 0, h - 1)
    # y1 = np.clip(y1, 0, h - 1)

    # Depth values at four neighbors
    d00 = depth_image[y0, x0] * 0.001
    d01 = depth_image[y0, x1] * 0.001
    d10 = depth_image[y1, x0] * 0.001
    d11 = depth_image[y1, x1] * 0.001   

    # Weights for bilinear interpolation
    w00 = (y1 - y) * (x1 - x) / ((x1 - x0) * (y1 - y0))
    w01 = (y1 - y) * (x - x0) / ((x1 - x0) * (y1 - y0))
    w10 = (y - y0) * (x1 - x) / ((x1 - x0) * (y1 - y0))
    w11 = (y - y0) * (x - x0) / ((x1 - x0) * (y1 - y0))

    # Interpolate
    z = w00 * d00 + w01 * d01 + w10 * d10 + w11 * d11
    # z = depth_image[y0, x] * 0.001

    # Convert pixel coordinates to camera frame coordinates
    x_camera = (x - px) * z / fx
    y_camera = (y - py) * z / fy

    return x_camera, y_camera, z

def camera_to_robot_base(x: float, y: float, z: float, r_camera_to_base: np.ndarray, t_camera_to_base: np.ndarray) -> tuple[float, float, float]:
    """
    Convert a 3D point (x, y, z) from the camera frame to the robot base frame.

    Args:
        x (float): X-coordinate in the camera frame.
        y (float): Y-coordinate in the camera frame.
        z (float): Z-coordinate in the camera frame.
        r_camera_to_base (numpy.ndarray): 3x3 rotation matrix from camera frame to robot base frame.
        t_camera_to_base (numpy.ndarray): 3x1 translation matrix from camera frame to robot base frame.

    Returns:
        tuple: A tuple (x, y, z) representing the 3D point in the robot base frame.
    """

    # Create a 4x1 vector for the 3D point in the camera frame
    point_camera = np.array([[x], [y], [z]], dtype=np.float64)

    # Transform the point to the robot base frame
    point_robot = np.matmul(r_camera_to_base, point_camera) + t_camera_to_base

    # Extract the x, y, z coordinates in the robot base frame
    x_robot, y_robot, z_robot = point_robot[0, 0], point_robot[1, 0], point_robot[2, 0]

    return x_robot, y_robot, z_robot

def reverse_transformation(rotation, translation):
    """
    Reverse the transformation from camera to board.
    Args:
        rotation (numpy.ndarray): Rotation matrix.
        translation (numpy.ndarray): Translation vector.
    Returns:
        tuple: Inverted rotation and translation.
    """
    rotation_inv = np.linalg.inv(rotation)
    translation_inv = -np.dot(rotation_inv, translation)
    return rotation_inv, translation_inv

def compute_board_to_camera(corners, chessboard_size: tuple[int, int], square_size: float, intrinsics: dict):
    obj_points = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    obj_points[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    obj_points *= square_size

    # Assuming intrinsic camera matrix and distortion coefficients are known
    camera_matrix = np.array([[intrinsics['fx'], 0, intrinsics['px']], [0, intrinsics['fy'], intrinsics['py']], [0, 0, 1]], dtype=np.float64)  # Example values
    dist_coeffs = np.array(intrinsics['coeffs'], dtype=np.float64)

    ret, rvec, tvec = cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)
    if not ret:
        raise Exception("Failed to compute board-to-camera transformation")
    return rvec, tvec

def find_chessboard_corners(image, chessboard_size):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if not ret:
        raise Exception("Chessboard corners not found")
    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    # cv2.drawChessboardCorners(image, chessboard_size, corners2, ret)
    # cv2.imshow('img', image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return corners2

class CameraClient():

    def __init__(self, ip_port: str) -> None:
        """Constructor.

        Args:
            ip_port: ip address and port of the socket.io server.
        """
        self.client = socketio.Client()
        self.client.on('connect', self.on_connect)
        self.client.on('connect_error', self.on_connect_error)
        self.client.on('disconnect', self.on_disconnect)
        self.client.on('broadcast_img', self.on_depth_color_img)
        self.color_view, self.depth_view, self.timestamp_view = None, None, time.time()
        # self.client.on('status', self.on_status)        

        self.client.connect(ip_port)
    
    def on_connect(self) -> None:
        print('Connectted to Realsense-d455 server. SSID: {self.client.sid}.')
    
    def on_connect_error(self, data) -> None:
        print('Connection to Realsense-d455 server failed.')

    def on_disconnect(self) -> None:
        print('Disconnected from Realsense-d455 server.')
    
    def get_camera_intrinsics(self):
        """Get camera intrinsics from the server."""
        data = self.client.call(event='intrinsics')
        return data

    def get_depth_color_img(self):
        """Get camera intrinsics from the server."""
        data = self.client.call(event='depth_color_img')
        return {'depth': pickle.loads(data['depth']), 'color': pickle.loads(data['color']), 'timestamp': data['timestamp'], 'msg': data['msg']}

    def get_sample_detection(self):
        """Get sample detection from the server."""
        data = self.client.call(event='sample_detection')
        return data

    def on_depth_color_img(self, data):
        """Get depth and color image from the server."""
        # self.client.emit('depth_color_img')
        # self.client.wait()
        # self.data = data
        # d = pickle.loads(data)
        d = data
        self.color_view, self.depth_view, self.timestamp_view = pickle.loads(d['color']), pickle.loads(d['depth']), d['timestamp']
        # self.color_view = pickle.loads
        self.time_receive = time.time()
        # print(self.timestamp_view)
        # print(self.time_receive)
        self.delay_ms = (self.time_receive - self.timestamp_view) * 1000
        # print(f'delay: {(self.time_receive - self.timestamp_view):4f} s' )
    
    def calibrate_gantry(self, chessboard_size: tuple[int, int] = (8, 6), square_size: float = 0.04) -> None:
        # camera = CameraClient('http://localhost:7627')
        intrinsics = self.get_camera_intrinsics()
        image_depth = self.get_depth_color_img()
        image = image_depth['color']
        depth = image_depth['depth']

        # Find chessboard corners
        corners = find_chessboard_corners(image, chessboard_size)
        # Compute transformation from chessboard to camera
        rvec, tvec = compute_board_to_camera(corners, chessboard_size, square_size, intrinsics)
        self.rotation = cv2.Rodrigues(rvec)[0]
        self.translation = np.reshape(tvec, (3, 1))

        x_error = []
        y_error = []
        z_error = []
        for i in range(corners.shape[0]):
            x_board, y_board, z_board = self.convert_pixel_to_board(corners[i][0][0], corners[i][0][1], depth, intrinsics)
            x_error.append(x_board - i % chessboard_size[0] * square_size)
            y_error.append(y_board - i // chessboard_size[0] * square_size)
            z_error.append(z_board - 0)

        print(f"x_error: {np.mean(x_error)}, {np.std(x_error)}")
        print(f"y_error: {np.mean(y_error)}, {np.std(y_error)}")
        print(f"z_error: {np.mean(z_error)}, {np.std(z_error)}")

        self.x_error = np.mean(x_error)
        self.y_error = np.mean(y_error)
        self.z_error = np.mean(z_error)

    def convert_pixel_to_board(self, x: float, y: float, depth_image: np.ndarray, camera_intrinsics: dict) -> tuple[float, float, float]:
        x_c, y_c, z_c = pixel_to_camera_frame(x, y, depth_image, camera_intrinsics)

        r_board_to_camera, t_board_to_camera = self.rotation, self.translation
        
        r_camera_to_board, t_camera_to_board = reverse_transformation(r_board_to_camera, t_board_to_camera)
        x_b, y_b, z_b = camera_to_robot_base(x_c, y_c, z_c, r_camera_to_board, t_camera_to_board)
        return x_b, y_b, z_b
    
    def convert_pixel_to_gantry(self, x: float, y: float) -> tuple[float, float, float]:
        intrinsics = self.get_camera_intrinsics()
        image_depth = self.get_depth_color_img()
        image = image_depth['color']
        depth = image_depth['depth']
        x_b, y_b, z_b = self.convert_pixel_to_board(x, y, depth, intrinsics)
        x_b, y_b, z_b = x_b - self.x_error, y_b - self.y_error, z_b - self.z_error

        x_g, y_g, z_g = x_b, y_b, z_b
        return x_g, y_g, z_g  

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start the RealSense D455 Socket.IO server.")
    parser.add_argument('--port', type=int, default=7673, help='Port to run the server on')
    args = parser.parse_args()
    port = args.port

    client = CameraClient(f'http://localhost:{port}')
    print(client.get_camera_intrinsics())
    while True:
        if client.color_view is not None:
            img = cv2.imdecode(client.color_view, flags = cv2.IMREAD_COLOR_BGR)

            text = f'Delay: {client.delay_ms:.1f} ms'
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            font_thickness = 2
            text_color = (0, 0, 255) # Black

            # Get image dimensions
            image_height, image_width, _ = img.shape

            # Calculate text size
            (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)

            # Calculate coordinates for the bottom-left corner of the text
            # x: image_width - text_width - margin
            # y: text_height + margin (since origin is top-left)
            margin = 10
            x_coordinate = image_width - text_width - margin
            y_coordinate = text_height + margin

            # Put the text on the image
            cv2.putText(img, text, (x_coordinate, y_coordinate), font, font_scale, text_color, font_thickness)

            cv2.imshow('color', img)
            
            cv2.waitKey(1)
    
    cv2.destroyAllWindows()