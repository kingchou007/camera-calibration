import pyrealsense2 as rs
import numpy as np

class RealSenseCamera():
    def __init__(self):
        super().__init__()
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        align = rs.align(rs.stream.color)
        self.pipeline.start(config)
        self.color_intrinsics = self.get_intrinsics("color")
        self.depth_intrinsics = self.get_intrinsics("depth")

    def get_color_intrinsics(self):
        return self.color_intrinsics

    def get_depth_intrinsics(self):
        return self.depth_intrinsics

    def get_color_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_depth_image(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image

    def get_intrinsics(self, camra_type):
        if camra_type == "color":
            profile = self.pipeline.get_active_profile()
            color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
            color_intrinsics = color_profile.get_intrinsics()
            K = [[color_intrinsics.fx, 0, color_intrinsics.ppx], [0, color_intrinsics.fy, color_intrinsics.ppy], [0, 0, 1]]
            return np.array(K)
        elif camra_type == "depth":≠
            profile = self.pipeline.get_active_profile()
            depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
            depth_intrinsics = depth_profile.get_intrinsics()
            K = [[depth_intrinsics.fx, 0, depth_intrinsics.ppx], [0, depth_intrinsics.fy, depth_intrinsics.ppy], [0, 0, 1]]
            return np.array(K)

if __name__ == "__main__":
    cam = RealSenseCamera()
    # while True:
        # color_image = cam.get_color_image()
        # depth_image = cam.get_depth_image()
        # cv2.imshow("color", color_image)
        # cv2.imshow("depth", depth_image)
        # cv2.waitKey(1)
    print(cam.get_color_intrinsics())
    print(cam.get_depth_intrinsics())
    time.sleep(1)