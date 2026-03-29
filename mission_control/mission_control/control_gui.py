import rclpy
from rclpy import executors
from rclpy.node import MutuallyExclusiveCallbackGroup, Node
import rclpy.utilities

import OpenGL.GL as gl
import glfw
import imgui.core as imgui
from imgui.integrations.glfw import GlfwRenderer
import colorsys

from sensor_msgs.msg import Image
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from std_msgs.msg import Float32MultiArray, Bool

from mission_control.gui.dashboard import Dashboard

from mission_control.config.gui import (
    WIDTH,
    HEIGHT,
)

import numpy as np
import cv2
import av
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory
import os

import subprocess
import glob
import re



def impl_glfw_init(window_name="Project Gorgon", width=WIDTH, height=HEIGHT):
    if not glfw.init():
        print("Could not initialize OpenGL context")
        exit(1)

    # OS X supports only forward-compatible core profiles from 3.2
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(int(width), int(height), window_name, None, None)
    glfw.make_context_current(window)

    if not window:
        glfw.terminate()
        print("Could not initialize Window")
        exit(1)

    return window


class GUI(Node):
    def __init__(self):
        super().__init__("control_gui")
        self.backgroundColor = (0, 0, 0, 1)
        self.window = impl_glfw_init()
        glfw.set_window_refresh_callback(self.window, self.window_refresh_CB_)
        glfw.set_framebuffer_size_callback(self.window, self.frame_buffer_size_CB_)

        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)

        # Window Size
        self.width = WIDTH
        self.height = HEIGHT
        
        self.qr_texID = gl.glGenTextures(1)

        self.dashboard = Dashboard(self.get_logger)

        # IMGUI
        imgui.get_io().font_global_scale = 1.2
        self.style = imgui.get_style()
        self.style.window_rounding = 3.0

        gui_group = MutuallyExclusiveCallbackGroup() 

        self.subscription_ = self.create_subscription(
            Image,
            "/picam_ros2/camera_88000/imx708_wide",
            self.image_unpack_effector,
            10,
            callback_group=gui_group
        )

        self.subscription_ = self.create_subscription(
            Image,
            "/picam_ros2/camera_80000/imx708_wide",
            self.image_unpack_main,
            10,
            callback_group=gui_group
        )


        # self.subscription = self.create_subscription(
        #     FFMPEGPacket,
        #     "/picam_ros2/camera_80000/imx708_wide_h264",
        #     self.ffmpeg_main,
        #     10
        # )
        #
        # self.subscription = self.create_subscription(
        #     FFMPEGPacket,
        #     "/picam_ros2/camera_88000/imx708_wide_h264",
        #     self.ffmpeg_effector,
        #     10
        # )


        self.img_effector = None
        self.img_main = None

        self.codec = av.CodecContext.create("h264", "r")
        self.buffer = b""

        self.position_subscriber_ = self.create_subscription(
                Float32MultiArray, 
                "/EffectorPositions", 
                self.store_targets, 
                10
        )

        self.position_publisher_gantry_ = self.create_publisher(
                Float32MultiArray, 
                "/Gantry",
                10)

        self.gantry_home_ = self.create_publisher(Bool, "/Home", 10)

        self.timer_ = self.create_timer(0.2, self.publish_gantry)


        self.target_effector_array = [0] * 4

        self.x = 50.0
        self.y = 50.0
        self.max_x = 100.0
        self.max_y = 100.0

        pkg_path = get_package_share_directory('mission_control')
        tool_path = os.path.join(pkg_path, 'tool_ncnn_model')
        elec_path = os.path.join(pkg_path, 'electronics_ncnn_model')
        
        # self.tool_model = YOLO(tool_path) 
        # self.elec_model = YOLO(elec_path)

    def publish_gantry(self):
        msg = Float32MultiArray()
        y = 250 - self.y * 2.5
        x = 560 - self.x * 5.6
        msg.data = [float(x), float(y)]

        self.position_publisher_gantry_.publish(msg)


    def store_targets(self, msg):
        self.target_effector_array = msg.data


    def ffmpeg_main(self, msg):
        try:
            self.buffer += msg.data

            packets = self.codec.parse(self.buffer)

            if packets:
                self.buffer = b""
    
            frames = None
            for packet in packets:
                frames = self.codec.decode(packet)
    
        except Exception as e:
            self.get_logger().error(f"Decode error: {e}")
    
    
    def process_results(self, results, model_name, color, annotated_frame):
        for r in results:
            annotated_frame = r.plot(img=annotated_frame) 
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(annotated_frame, (center_x, center_y), 5, color, -1)

        return annotated_frame


    def ffmpeg_effector(self, msg):
        try:
            packet = av.Packet(msg.data)

            frames = self.codec.decode(packet)

            for frame in frames:
                self.img_effector = frame.to_ndarray(format='bgr24')

        except Exception as e:
            self.get_logger().error(f"Decode error: {e}")
        

    def image_unpack_effector(self, msg: Image):

        h, w = msg.height, msg.width
        expected_size = int(w * h * 1.5)
        yuv = np.frombuffer(msg.data[:expected_size], dtype=np.uint8)

        # # Reshape to (height * 3/2, width)
        yuv = yuv.reshape((int(h * 1.5), w))

        # Convert to BGR using OpenCV
        # cv2.COLOR_YUV2BGR_I420 works for standard YUV420 planar
        self.img_effector = cv2.cvtColor(yuv, cv2.COLOR_YUV420P2BGR)


    def image_unpack_main(self, msg: Image):
        h, w = msg.height, msg.width
        expected_size = int(w * h * 1.5)
        # yuv = np.frombuffer(msg.data[:expected_size], dtype=np.uint8)
        yuv = np.frombuffer(msg.data[:expected_size], dtype=np.uint8)

        # # Reshape to (height * 3/2, width)
        yuv = yuv.reshape((int(h * 1.5), w))

        # Convert to BGR using OpenCV
        # cv2.COLOR_YUV2BGR_I420 works for standard YUV420 planar
        self.img_main = cv2.cvtColor(yuv, cv2.COLOR_YUV420P2BGR)

      
        # if self.img_main is not None:
        #     annotated_frame = self.img_main.copy()
        #
        #     tool_results = self.tool_model(self.img_main, verbose=False, imgsz=320)
        #     # elec_results = self.elec_model(self.img_main, verbose=False, imgsz=320)
        #
        #
        #     self.img_main = self.process_results(tool_results, "TOOL", (0, 255, 0), annotated_frame)
        #     # self.process_results(elec_results, "ELEC", (255, 0, 0), annotated_frame)
    


    def window_refresh_CB_(self, window):
        self.run()
        gl.glFinish()

    def frame_buffer_size_CB_(self, window, width, height):
        gl.glViewport(0, 0, width, height)
        self.width = width
        self.height = height

        scale_change = 0.5 * (((width) / (WIDTH) * 1.2) + ((height) / (HEIGHT) * 1.2))

        ARBRITRARY_SCALE = 0.4
        imgui.get_io().font_global_scale = (
            1.2 - ARBRITRARY_SCALE + scale_change * ARBRITRARY_SCALE
        )

    def bind_image(self, img):
        image = np.array(img)
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.qr_texID)

        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

        # Set texture clamping method
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)

        gl.glTexImage2D(
            gl.GL_TEXTURE_2D,
            0,
            gl.GL_RGB,
            image.shape[1],
            image.shape[0],
            0,
            gl.GL_RGB,
            gl.GL_UNSIGNED_BYTE,
            image,
        )

    def run(self):
        # while not glfw.window_should_close(self.window):
        glfw.poll_events()
        self.impl.process_inputs()
        gl.glClearColor(*self.backgroundColor)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
   
        if self.img_main is not None and self.img_effector is not None:
            self.dashboard.draw([self.img_main, self.img_effector])
    
        # IMGUI - BEGIN ---
        imgui.new_frame()
    
        # PERFORMANCE
        imgui.begin("Performance")
        io = imgui.get_io()
        imgui.text(f"FPS: {io.framerate:.2f}")
        imgui.end()
 
        # MAIN DASHBOARD
        # offset = self.width / 40
        offset = 0
        imgui.set_next_window_position(self.width / 2 + self.width/10 + offset, 0)
        imgui.set_next_window_size(self.width / 2 - self.width/10- offset, self.height)
 
        imgui.begin(
        "Dashboard",
        flags=imgui.WINDOW_NO_COLLAPSE
        | imgui.WINDOW_NO_MOVE
        | imgui.WINDOW_NO_RESIZE
        | imgui.WINDOW_MENU_BAR
        | imgui.WINDOW_NO_BRING_TO_FRONT_ON_FOCUS,
        )
 
        # TELEMETRY
 
        imgui.same_line(position=self.width * 9 / 32)
 
        # BATTERY MONITOR
        imgui.begin_group()
        imgui.text("Playback Progress:")
        # from 0 to 255 -> OpenGL and ImGui uses float values to conversion is needed
        # Magic Colours ->
        low_bat_colour = (181, 56, 56)
        high_bat_colour = (31, 161, 91)
 
        batt_colour_rgb = colour_interpolate(
        low_bat_colour, high_bat_colour, 0, linear
        )
        display_batt_colour = normalise_rgb(batt_colour_rgb)
 
        imgui.push_style_color(
            imgui.COLOR_PLOT_HISTOGRAM,
            *display_batt_colour,
        )
        imgui.progress_bar(
            0, (self.width / 14, 18 + 1 / 200 * self.height), ""
        )
        imgui.pop_style_color(1)
        imgui.same_line()
        imgui.text(f"{0 * 100:.1f}%")
        imgui.end_group()
    
        imgui.spacing()

        imgui.begin_group()
        
        imgui_text = [
            "Telemetry:",
            "(All data is in degrees, and m/s)\n\n",
            "Shoulder: {:.3f}\n".format(self.target_effector_array[0]),
            "Elbow: {:.3f}\n".format(self.target_effector_array[1]),
            "Wrist: {:.3f}\n".format(self.target_effector_array[2]),
            "Claw: {:.3f}\n".format(self.target_effector_array[3]),
        ]

        for text in imgui_text:
            imgui.text(text)

        imgui.end_group()

        imgui.begin_group()

        changed_x, self.x = imgui.slider_float("X", self.x, 0, self.max_x)
        changed_y, self.y = imgui.slider_float("Y", self.y, 0, self.max_y)

        _, new_x = imgui.input_float("Set X", self.x, step=1.0)
        _, new_y = imgui.input_float("Set Y", self.y, step=1.0)
        self.x, self.y = new_x, new_y

        self.clamp()

        imgui.separator()

        # --- Canvas ---
        avail_w, avail_h = imgui.get_content_region_available()

        canvas_height = avail_h * 0.35   # 50% of remaining space
        canvas_size = (avail_w, canvas_height)

        imgui.invisible_button("canvas", canvas_size[0], canvas_size[1])


        draw_list = imgui.get_window_draw_list()
        p0 = imgui.get_item_rect_min()
        p1 = imgui.get_item_rect_max()

        # Background
        draw_list.add_rect_filled(*p0, *p1, imgui.get_color_u32_rgba(0.1, 0.1, 0.1, 1))

        # Grid
        grid_spacing = 20
        for x in range(int(p0[0]), int(p1[0]), grid_spacing):
            draw_list.add_line(x, p0[1], x, p1[1], imgui.get_color_u32_rgba(0.3, 0.3, 0.3, 1))
        for y in range(int(p0[1]), int(p1[1]), grid_spacing):
            draw_list.add_line(p0[0], y, p1[0], y, imgui.get_color_u32_rgba(0.3, 0.3, 0.3, 1))

        # Convert coords
        def to_screen(gx, gy):
            sx = p0[0] + (gx / self.max_x) * (p1[0] - p0[0])
            sy = p1[1] - (gy / self.max_y) * (p1[1] - p0[1])
            return sx, sy

        # Draw point
        sx, sy = to_screen(self.x, self.y)
        draw_list.add_circle_filled(sx, sy, 6, imgui.get_color_u32_rgba(1, 0, 0, 1))

        # Interaction (NOW WORKS PROPERLY)
        io = imgui.get_io()
        if imgui.is_item_hovered() and imgui.is_mouse_down(0):
            mx, my = io.mouse_pos
            gx = (mx - p0[0]) / (p1[0] - p0[0]) * self.max_x
            gy = (p1[1] - my) / (p1[1] - p0[1]) * self.max_y
            self.x = gx
            self.y = gy
            self.clamp()

        imgui.text(f"Position: X={self.x:.2f}, Y={self.y:.2f}")

        imgui.end_group()

        imgui.begin_group()
        if imgui.button("Begin Homing Sequence"):
            msg = Bool()
            msg.data = True
            self.gantry_home_.publish(msg)

        if imgui.button("Open Object Detect"):
            pkg_path = get_package_share_directory('mission_control')
            subprocess.Popen(["python3", os.path.join(pkg_path, 'ncnn_model/open_live_preview.py'), "--path", os.path.join(pkg_path, 'ncnn_model')])


        if imgui.button("Home to Lasted Object"):
            if extract_x_from_latest() is not None:
                self.x = extract_x_from_latest()[0]


        imgui.end_group()

        imgui.end()
 
        imgui.render()
        # END OF IMGUI -----------
 
        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)

    def clamp(self):
        self.x = max(0, min(self.x, self.max_x))
        self.y = max(0, min(self.y, self.max_y))



def extract_x_from_latest():
    files = glob.glob("detections_*.txt")
    
    if not files:
        print("No files found")
        return None
    
    latest_file = max(files, key=os.path.getmtime)
    return extract_x_coordinates(latest_file)


def colour_interpolate(c1, c2, percentage_c2, interpolator):
    hsv_c1 = colorsys.rgb_to_hsv(*c1)
    hsv_c2 = colorsys.rgb_to_hsv(*c2)

    final = (
        interpolator(hsv_c1[0], hsv_c2[0], percentage_c2),
        interpolator(hsv_c1[1], hsv_c2[1], percentage_c2),
        interpolator(hsv_c1[2], hsv_c2[2], percentage_c2),
    )

    return colorsys.hsv_to_rgb(*final)


def linear(v1, v2, percentage_v2):
    return v1 * (1 - percentage_v2) + v2 * percentage_v2


def normalise_rgb(rgb):
    r, g, b = rgb
    return (r / 255, g / 255, b / 255)

def extract_x_coordinates(file_path):
    x_values = []

    with open(file_path, "r") as f:
        for line in f:
            match = re.search(r"Center:\s*\((\d+),\s*(\d+)\)", line)
            if match:
                x = int(match.group(1))
                x_values.append(x)

    return x_values


def main(args=None):
    rclpy.init(args=args)
    
    gui = GUI()
    
    try:
        # # Run GUI
        while not glfw.window_should_close(gui.window):
            rclpy.spin_once(gui, timeout_sec=0.02)
            # rclpy.spin_once(gui)
            gui.run()
    except:
        # Cleanup After Shutdown
        gui.impl.shutdown()
        glfw.terminate()
        rclpy.utilities.try_shutdown()
