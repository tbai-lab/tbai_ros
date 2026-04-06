#!/usr/bin/env python3

import time

import depthai as dai
import numpy as np
import ros_numpy
import rospy
import scipy.spatial.transform
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def create_pointcloud2(odometry, point_cloud):
  """
  Converts a numpy array of shape [N, 3] representing a point cloud in the camera frame
  into a PointCloud2 message in the odom frame.

  Args:
      odometry (np.ndarray): 4x4 transformation matrix from odom frame to camera frame.
      point_cloud (np.ndarray): Nx3 array of (x, y, z) coordinates in the camera frame.

  Returns:
      sensor_msgs.msg.PointCloud2: The transformed point cloud message.
  """
  if point_cloud.ndim != 2 or point_cloud.shape[1] != 3:
    raise ValueError("Point cloud must be a numpy array of shape (N, 3)")

  if odometry.shape != (4, 4):
    raise ValueError("Odometry must be a 4x4 numpy array")

  rospy.logwarn(f"Point cloud shape: {point_cloud.shape}")
  point_cloud = point_cloud[::7, ...]
  N = point_cloud.shape[0]

  if N == 0:
    # Empty point cloud
    structured = np.array([], dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
  else:
    # Transform points from camera to odom frame
    # Rotation around Y by pi/2
    Ry = scipy.spatial.transform.Rotation.from_euler("y", np.pi / 2).as_matrix()
    Rz = scipy.spatial.transform.Rotation.from_euler("z", -np.pi / 2).as_matrix()
    T_camera_to_odom = np.linalg.inv(Rz @ Ry)
    T = np.eye(4)
    T[:3, :3] = T_camera_to_odom

    # Add homogeneous coordinate
    ones = np.ones((N, 1), dtype=point_cloud.dtype)
    points_hom = np.hstack((point_cloud, ones))

    # Apply transformation
    points_odom_hom = np.dot(T, points_hom.T).T
    points_odom = points_odom_hom[:, :3]

    # Create structured array
    structured = np.zeros(N, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
    structured["x"] = -points_odom[:, 0].astype(np.float32)
    structured["y"] = -points_odom[:, 1].astype(np.float32)
    structured["z"] = points_odom[:, 2].astype(np.float32)

  # Create PointCloud2 message
  stamp = rospy.Time.now()
  frame_id = "camera_frame"
  pc2_msg = ros_numpy.msgify(PointCloud2, structured, stamp=stamp, frame_id=frame_id)

  return pc2_msg


def main():
  rospy.init_node("dummy_mapping", anonymous=False)

  # Initialize CV bridge
  bridge = CvBridge()

  # Create ROS publishers
  image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
  camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1)
  pointcloud_pub = rospy.Publisher("/camera/pointcloud", PointCloud2, queue_size=1)

  # Initialize tf broadcaster
  tf_broadcaster = tf2_ros.TransformBroadcaster()

  # Create camera info with dummy values
  camera_info = CameraInfo()
  camera_info.header.frame_id = "camera_frame"
  camera_info.width = 640
  camera_info.height = 400
  camera_info.distortion_model = "plumb_bob"
  camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
  camera_info.K = [500.0, 0.0, 320.0, 0.0, 500.0, 200.0, 0.0, 0.0, 1.0]
  camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  camera_info.P = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 200.0, 0.0, 0.0, 0.0, 1.0, 0.0]

  # Create pipeline with VIO and point cloud generation
  with dai.Pipeline() as p:
    fps = 30
    width = 640
    height = 400

    # Define sources and outputs
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    color = p.create(dai.node.Camera)
    imu = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()

    # Configure color camera
    color.build()

    # Configure stereo depth
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.initialConfig.postProcessing.thresholdFilter.maxRange = 10000
    rgbd.setDepthUnits(dai.StereoDepthConfig.AlgorithmControl.DepthUnit.METER)

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking for VIO
    left_output = left.requestOutput((width, height))
    right_output = right.requestOutput((width, height))
    left_output.link(odom.left)
    right_output.link(odom.right)
    imu.out.link(odom.imu)

    # Linking for stereo depth and point cloud
    left_output.link(stereo.left)
    right_output.link(stereo.right)

    # Handle different platforms for color alignment
    platform = p.getDefaultDevice().getPlatform()
    if platform == dai.Platform.RVC4:
      color_output = color.requestOutput((width, height), dai.ImgFrame.Type.RGB888i)
      align = p.create(dai.node.ImageAlign)
      stereo.depth.link(align.input)
      color_output.link(align.inputAlignTo)
      align.outputAligned.link(rgbd.inDepth)
    else:
      color_output = color.requestOutput((width, height), dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 30, True)
      stereo.depth.link(rgbd.inDepth)
      color_output.link(stereo.inputAlignTo)
    color_output.link(rgbd.inColor)

    # Create output queues
    passthrough_queue = odom.passthrough.createOutputQueue()
    transform_queue = odom.transform.createOutputQueue()
    pointcloud_queue = rgbd.pcl.createOutputQueue()

    p.start()

    rospy.loginfo("DepthAI VIO pipeline started")

    while p.isRunning() and not rospy.is_shutdown():
      # Get VIO transform data
      trans_data = transform_queue.tryGet()
      img_frame = passthrough_queue.tryGet()
      pcl_data = pointcloud_queue.tryGet()

      if trans_data is not None and img_frame is not None:
        # Get VIO transform
        trans = trans_data.getTranslation()
        quat = trans_data.getQuaternion()

        # Create transformation matrix from VIO data
        T_odom_to_camera = np.eye(4)
        T_odom_to_camera[:3, 3] = [trans.x, trans.y, trans.z]
        # Convert quaternion to rotation matrix (simplified - assuming identity for now)
        # TODO: Implement proper quaternion to rotation matrix conversion

        # Get OpenCV frame
        cv_image = img_frame.getCvFrame()

        # Convert to ROS Image message with proper encoding
        if len(cv_image.shape) == 3:
          ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        else:
          ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
        current_time = rospy.Time.now()
        ros_image.header.stamp = current_time
        ros_image.header.frame_id = "camera_frame"

        # Update camera info timestamp
        camera_info.header.stamp = current_time

        # Create and publish transform from odom to camera_frame using VIO data
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "odom"
        transform.child_frame_id = "camera_frame"
        transform.transform.translation.x = trans.x
        transform.transform.translation.y = trans.y
        transform.transform.translation.z = trans.z
        transform.transform.rotation.x = quat.qx
        transform.transform.rotation.y = quat.qy
        transform.transform.rotation.z = quat.qz
        transform.transform.rotation.w = quat.qw
        tf_broadcaster.sendTransform(transform)

        # Rate limiter for publishing at 2 Hz
        if not hasattr(main, "last_pub_time"):
          main.last_pub_time = 0.0
        now = time.time()
        if now - main.last_pub_time >= 0.5 or True:
          # Process and publish point cloud if available
          if pcl_data is not None:
            try:
              points, colors = pcl_data.getPointsRGB()
              rospy.loginfo(f"Points: {len(points)}")
              if len(points) > 0:
                # Convert to numpy array
                points_np = np.array(points)
                # Create and publish PointCloud2 message
                pc2_msg = create_pointcloud2(T_odom_to_camera, points_np)
                pointcloud_pub.publish(pc2_msg)
            except Exception as e:
              rospy.logwarn(f"Failed to process point cloud: {e}")

          # Publish the image and camera info
          main.last_pub_time = now
        camera_info_pub.publish(camera_info)
        image_pub.publish(ros_image)

        rospy.loginfo("Published image, VIO transform, and point cloud")

      time.sleep(0.01)


if __name__ == "__main__":
  main()
