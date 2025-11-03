import socketio
import pickle
import cv2
import time
import argparse

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