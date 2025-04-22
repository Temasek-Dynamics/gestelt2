# ROS2
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import *

from cv_bridge import CvBridge

import ecal.core.core as ecal_core

# capnp
from ament_index_python.packages import get_package_share_directory
capnp_path = get_package_share_directory('vilota_bridge') + '/capnp/'

import capnp
capnp.add_import_hook([capnp_path])

from vilota_bridge.capnp_subscriber import CapnpSubscriber
import disparity_capnp as eCALDiaprity # type: ignore
import image_capnp as eCALImage # type: ignore
import odometry3d_capnp as eCALOdometry3d # type: ignore

import cv2
import numpy as np

import multiprocessing

# Abstract away all the ROS stuff
class VilotaBridge(Node):
    def publishDepth(self, currentDepth):
        currentTimestamp = self.get_clock().now().to_msg()

        imageMsg = self.cvBridge.cv2_to_imgmsg(currentDepth)
        imageMsg.encoding = "32FC1"
        imageMsg.header.frame_id = self.cameraFrameID
        imageMsg.header.stamp = currentTimestamp
        self.depthPublisher.publish(imageMsg)

    def getDepthImage(self, disparityMsg):
        MAX_DEPTH = 20

        mat_uint16 = np.frombuffer(disparityMsg.data, dtype=np.uint16)
        mat_uint16 = mat_uint16.reshape((disparityMsg.height, disparityMsg.width, 1))

        mat_float32 = mat_uint16.astype(np.float32) / 8.0
        mat_float32 = cv2.medianBlur(mat_float32, 5) # 5x5 median filter
        # mat_float32 = cv2.bilateralFilter(mat_float32, 9, 75, 75) # 9x9 bilateral filter

        # Compute depth array
        # Assuming disparityMsg contains baseline and fx information
        min_disparity_threshold = 1e-5  # Adjust this threshold based on your scene characteristics

        # Calculate depth, limiting the minimum disparity+
        epsilon = 1e-1 # Huimin says this will let the error be more conservative compared to 1e-5
        # epsilon = 1e-5
        depth = np.where(mat_float32 > min_disparity_threshold + epsilon,
                        (disparityMsg.baseline * disparityMsg.fx / (mat_float32 + epsilon)),
                        MAX_DEPTH)

        # Limit the maximum depth
        depth = np.minimum(depth, MAX_DEPTH)

        return depth

    def getCameraInfo(self, disparityMsg):
        cameraInfoMsg = CameraInfo()
        cameraInfoMsg.width = disparityMsg.width
        cameraInfoMsg.height = disparityMsg.height
        cameraInfoMsg.header.frame_id = self.cameraFrameID

        # Intrinsic camera matrix for the raw (distorted) images.
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        # Projects 2D points in the camera coordinate frame to 3D pixel
        # coordinates using the focal lengths (fx, fy) and principal point
        # (cx, cy).

        # camera info from disparity is based on 640x400 image, not disparity, so we need to divide
        fx = disparityMsg.fx / 2.0
        fy = disparityMsg.fy / 2.0
        cx = disparityMsg.cx / 2.0
        cy = disparityMsg.cy / 2.0

        cameraInfoMsg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        return cameraInfoMsg

    def publishCameraInfo(self, cameraInfoMsg):
        cameraInfoMsg.header.stamp = self.get_clock().now().to_msg()
        self.cameraInfoPublisher.publish(cameraInfoMsg)

    def processDepth(self, running, bridgeName, disparityTopic):
        # Init ecal and subscriptions
        ecal_core.initialize([], f"[{bridgeName}] {disparityTopic}")
        self.get_logger().info(f"[{bridgeName}] Subscribing from eCal {disparityTopic}")
        disparitySub = CapnpSubscriber("Disparity", disparityTopic)

        while running.value and ecal_core.ok():
            ret, disparityMsg, _ = disparitySub.receive(1) # timeout/sleep for 1 ms

            if not ret:
                continue

            with eCALDiaprity.Disparity.from_bytes(disparityMsg) as disparityMsg:
                # disparity16 @1; # encoded as scaled disparity of scaling of 8
                assert(disparityMsg.encoding == "disparity16")

                depthImage = self.getDepthImage(disparityMsg)
                self.publishDepth(depthImage)

                cameraInfoMsg = self.getCameraInfo(disparityMsg)
                self.publishCameraInfo(cameraInfoMsg)

        ecal_core.finalize()

    def processOdometry(self, running, bridgeName, odometryTopic, dynamicTransformsQueue):
        # ROS stuff
        odomToBaseLinkTF = TransformStamped()
        odomToBaseLinkTF.header.frame_id = "odom"
        odomToBaseLinkTF.child_frame_id = "base_link"

        # Init ecal and subscriptions
        ecal_core.initialize([], f"[{bridgeName}] {odometryTopic}")
        self.get_logger().info(f"[{bridgeName}] Subscribing from eCal {odometryTopic}")

        odometrySub = CapnpSubscriber("Odometry3d", odometryTopic)

        while running.value and ecal_core.ok():
            # wait for message to come in
            ret, msg, _ = odometrySub.receive(1) # timeout/sleep for 1 ms

            if not ret:
                continue

            # Odom pose is NWU, master unit faces backwards
            with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:
                # translation/position
                odomToBaseLinkTF.transform.translation.x = odometryMsg.pose.position.x
                odomToBaseLinkTF.transform.translation.y = odometryMsg.pose.position.y
                odomToBaseLinkTF.transform.translation.z = odometryMsg.pose.position.z

                # rotation/orientation
                odomToBaseLinkTF.transform.rotation.x = odometryMsg.pose.orientation.x
                odomToBaseLinkTF.transform.rotation.y = odometryMsg.pose.orientation.y
                odomToBaseLinkTF.transform.rotation.z = odometryMsg.pose.orientation.z
                odomToBaseLinkTF.transform.rotation.w = odometryMsg.pose.orientation.w

                # Send the transformation
                odomToBaseLinkTF.header.stamp = self.get_clock().now().to_msg()
                dynamicTransformsQueue.put(odomToBaseLinkTF)

        ecal_core.finalize()

    def publishImage(self, currentImage):
        currentImage = cv2.cvtColor(currentImage, cv2.COLOR_BGR2GRAY)

        currentTimestamp = self.get_clock().now().to_msg()
        imageMsg = self.cvBridge.cv2_to_imgmsg(currentImage)
        imageMsg.encoding = "mono8"
        imageMsg.header.frame_id = self.cameraFrameID
        imageMsg.header.stamp = currentTimestamp
        self.imagePublisher.publish(imageMsg)

    def processImage(self, running, bridgeName, imageTopic, staticTransformsQueue):
        ecal_core.initialize([], f"[{bridgeName}] {imageTopic}")
        self.get_logger().info(f"[{bridgeName}] Subscribing from eCal {imageTopic}")
        imageSub = CapnpSubscriber("Image", imageTopic)

        while running.value and ecal_core.ok():
            # wait for message to come in
            ret, msg, _ = imageSub.receive(1) # timeout/sleep for 1 ms

            if not ret:
                continue

            with eCALImage.Image.from_bytes(msg) as imageMsg:
                assert(imageMsg.encoding == "jpeg")

                currentImageJpeg = np.frombuffer(imageMsg.data, dtype=np.uint8)
                currentImage = cv2.imdecode(currentImageJpeg, cv2.IMREAD_COLOR)

                self.publishImage(currentImage)
                self.publishBaseLinkToCamera(bridgeName, staticTransformsQueue, imageMsg.extrinsic.bodyFrame)

        ecal_core.finalize()

    def publishBaseLinkToCamera(self, bridgeName, staticTransformsQueue, transform):
        # Should be FRD
        baseLinkToCameraTF = TransformStamped()

        baseLinkToCameraTF.header.stamp = self.get_clock().now().to_msg()
        baseLinkToCameraTF.header.frame_id = "base_link" # body
        baseLinkToCameraTF.child_frame_id = self.cameraFrameID

        baseLinkToCameraTF.transform.translation.x = transform.position.x
        baseLinkToCameraTF.transform.translation.y = transform.position.y
        baseLinkToCameraTF.transform.translation.z = transform.position.z

        baseLinkToCameraTF.transform.rotation.x = transform.orientation.x
        baseLinkToCameraTF.transform.rotation.y = transform.orientation.y
        baseLinkToCameraTF.transform.rotation.z = transform.orientation.z
        baseLinkToCameraTF.transform.rotation.w = transform.orientation.w

        staticTransformsQueue.put(baseLinkToCameraTF)

    def publishStaticTransforms(self):
        currentTimestamp = self.get_clock().now().to_msg()
        self.noChangeTF.header.stamp = currentTimestamp

        # https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
        # world -> map -> odom -> base_link -> depth_xxx
        self.noChangeTF.header.frame_id = "world"
        self.noChangeTF.child_frame_id = "map"
        self.staticTransformBroadcaster.sendTransform(self.noChangeTF)

        self.noChangeTF.header.frame_id = "map"
        self.noChangeTF.child_frame_id = "odom"
        self.staticTransformBroadcaster.sendTransform(self.noChangeTF)

        while not self.staticTransformsQueue.empty():
            currentStaticTransform = self.staticTransformsQueue.get()
            self.staticTransformBroadcaster.sendTransform(currentStaticTransform)

    def publishDynamicTransforms(self):
        while not self.dynamicTransformsQueue.empty():
            currentDynamicTransform = self.dynamicTransformsQueue.get()
            self.transformBroadcaster.sendTransform(currentDynamicTransform)

    def publishCameraPose(self, bridgeName):
        transformFromFrame = "depth_" + bridgeName
        transformToFrame = "odom"

        if not self.transformBuffer.can_transform(transformToFrame, transformFromFrame, rclpy.time.Time()):
            return

        t = self.transformBuffer.lookup_transform(transformToFrame, transformFromFrame, rclpy.time.Time())

        # Update camera pose
        cameraPoseMsg = PoseStamped()
        cameraPoseMsg.header.frame_id = "odom"
        cameraPoseMsg.header.stamp = self.get_clock().now().to_msg()

        cameraPoseMsg.pose.position.x = t.transform.translation.x
        cameraPoseMsg.pose.position.y = t.transform.translation.y
        cameraPoseMsg.pose.position.z = t.transform.translation.z

        cameraPoseMsg.pose.orientation.x = t.transform.rotation.x
        cameraPoseMsg.pose.orientation.y = t.transform.rotation.y
        cameraPoseMsg.pose.orientation.z = t.transform.rotation.z
        cameraPoseMsg.pose.orientation.w = t.transform.rotation.w

        self.posePublisher.publish(cameraPoseMsg)

    def initPublishers(self, bridgeName):
        self.get_logger().info(f"[Bridge] {bridgeName}")

        depthPublisherTopic = f"/{bridgeName}/depth/rect"
        self.get_logger().info(f"Publishing to ROS {depthPublisherTopic}")
        self.depthPublisher = self.create_publisher(Image, depthPublisherTopic, qos_profile_sensor_data)
        self.create_publisher

        cameraInfoPublisherTopic = f"/{bridgeName}/depth/camera_info"
        self.get_logger().info(f"Publishing to ROS {cameraInfoPublisherTopic}")
        self.cameraInfoPublisher = self.create_publisher(CameraInfo, cameraInfoPublisherTopic, qos_profile_sensor_data)

        posePublisherTopic = f"/{bridgeName}/depth/pose"
        self.get_logger().info(f"Publishing to ROS {posePublisherTopic}")
        self.posePublisher = self.create_publisher(PoseStamped, posePublisherTopic, qos_profile_sensor_data)

        imagePublisherTopic = f"/{bridgeName}/image/rect"
        self.get_logger().info(f"Publishing to ROS {imagePublisherTopic}")
        self.imagePublisher = self.create_publisher(Image, imagePublisherTopic, qos_profile_sensor_data)

    def __init__(self, running, bridgeName, disparityTopic, imageTopic, odomTopic):
        super().__init__(f"depth_publisher_{bridgeName}")

        # Create all ROS publisher/subscriber/callbacks in same context to prevent problems
        self.initPublishers(bridgeName)
        self.timerCallbackGroup = ReentrantCallbackGroup() # run all timers in parallel

        # OpenCV
        self.cvBridge = CvBridge()

        # Transforms queues
        # multiprocessing doesn't play well with ROS transform publishers
        # so we need them all on the same ROS context
        TRANSFORMS_PUBLISH_INTERVAL = 1.0 / 16.0 # seconds

        self.staticTransformsQueue = multiprocessing.Queue()
        self.dynamicTransformsQueue = multiprocessing.Queue()

        # Static transforms
        self.noChangeTF = TransformStamped()

        self.noChangeTF.transform.translation.x = 0.0
        self.noChangeTF.transform.translation.y = 0.0
        self.noChangeTF.transform.translation.z = 0.0

        self.noChangeTF.transform.rotation.x = 0.0
        self.noChangeTF.transform.rotation.y = 0.0
        self.noChangeTF.transform.rotation.z = 0.0
        self.noChangeTF.transform.rotation.w = 1.0
        self.staticTransformBroadcaster = StaticTransformBroadcaster(self)
        self.staticTransformsTimer = self.create_timer(TRANSFORMS_PUBLISH_INTERVAL, self.publishStaticTransforms, callback_group=self.timerCallbackGroup)

        # Dynamic transforms
        self.transformBroadcaster = TransformBroadcaster(self)
        self.transformBuffer = Buffer()
        self.transformListener = TransformListener(self.transformBuffer, self)

        self.dynamicTransformsTimer = self.create_timer(TRANSFORMS_PUBLISH_INTERVAL, self.publishDynamicTransforms, callback_group=self.timerCallbackGroup)
        self.cameraPoseTimer = self.create_timer(TRANSFORMS_PUBLISH_INTERVAL, lambda: self.publishCameraPose(bridgeName), callback_group=self.timerCallbackGroup)

        # Camera link
        self.cameraFrameID = "camera_" + bridgeName

        # eCal subscriptions
        self.imageProcess = multiprocessing.Process(target=self.processImage, args=(running, bridgeName, imageTopic, self.staticTransformsQueue))
        self.imageProcess.start()

        self.depthProcess = multiprocessing.Process(target=self.processDepth, args=(running, bridgeName, disparityTopic))
        self.depthProcess.start()

        self.odometryProcess = multiprocessing.Process(target=self.processOdometry, args=(running, bridgeName, odomTopic, self.dynamicTransformsQueue))
        self.odometryProcess.start()