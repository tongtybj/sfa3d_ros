#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
#from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import rospkg
import numpy as np
import timeit

import sys
import os
import time
import warnings
import zipfile

warnings.filterwarnings("ignore", category=UserWarning)

sys.path.append(os.path.join(os.path.dirname(__file__), '../SFA3D'))

import cv2
import torch

from sfa.models.model_utils import create_model
from sfa.utils.evaluation_utils import draw_predictions, convert_det_to_real_values
import sfa.config.kitti_config as cnf
from sfa.data_process.transformation import lidar_to_camera_box
from sfa.utils.visualization_utils import merge_rgb_to_bev, show_rgb_image_with_boxes
from sfa.data_process.kitti_data_utils import Calibration
from sfa.utils.demo_utils import parse_demo_configs, do_detect, download_and_unzip, write_credit
from sfa.data_process.kitti_bev_utils import makeBEVMap
import sfa.config.kitti_config as cnf
from sfa.data_process.kitti_data_utils import get_filtered_lidar

ID_TO_CLASS_NAME = {
    0: 'pedestrian',
    1: 'car',
    2: 'cyclist',
    -3: 'truck',
    -99: 'tram',
    -1: 'unknown'
}

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def on_scan(scan):
    start = timeit.default_timer()
    rospy.logdebug("Got scan")
    gen = []
    for p in pc2.read_points(scan, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
        gen.append(np.array([p[0], p[1], p[2], p[3]/100.0]))
    gen_numpy = np.array(gen, dtype=np.float32)

    front_lidar = get_filtered_lidar(gen_numpy, cnf.boundary)
    bev_map = makeBEVMap(front_lidar, cnf.boundary)
    bev_map = torch.from_numpy(bev_map)

    with torch.no_grad():
        detections, bev_map, fps = do_detect(configs, model, bev_map, is_front=True)

    objects_msg = MarkerArray()
    flag = False
    for j in range(configs.num_classes):
        class_name = ID_TO_CLASS_NAME[j]

        if len(detections[j]) > 0:
            flag = True
            marker_id = 0;
            for det in detections[j]:
                _score, _x, _y, _z, _h, _w, _l, _yaw = det
                yaw = -_yaw
                x = _y / cnf.BEV_HEIGHT * cnf.bound_size_x + cnf.boundary['minX']
                y = _x / cnf.BEV_WIDTH * cnf.bound_size_y + cnf.boundary['minY']
                z = _z + cnf.boundary['minZ']
                w = _w / cnf.BEV_WIDTH * cnf.bound_size_y
                l = _l / cnf.BEV_HEIGHT * cnf.bound_size_x
                obj = Marker()
                obj.header.stamp = rospy.Time.now()
                obj.header.frame_id = scan.header.frame_id
                obj.type = Marker.CUBE
                obj.action = Marker.ADD
                marker_id += 1
                obj.id = marker_id
                obj.lifetime = rospy.Duration.from_sec(0.2) # heuristic

                obj.pose.position.x = x
                obj.pose.position.y = y
                obj.pose.position.z = z
                [qx, qy, qz, qw] =  euler_to_quaternion(yaw, 0, 0)
                obj.pose.orientation.x = qx
                obj.pose.orientation.y = qy
                obj.pose.orientation.z = qz
                obj.pose.orientation.w = qw

                obj.scale.x = l
                obj.scale.y = w
                obj.scale.z = _h

                ## heuristic color setting
                if j == 0: # human
                    obj.color.r = 1.0
                    obj.color.a = 1.0
                else:
                    obj.color.b = 1.0
                    obj.color.a = 1.0

                objects_msg.markers.append(obj)

    if flag is True:
        pub.publish(objects_msg)

    stop = timeit.default_timer()
    rospy.loginfo('total time: {:.3f} [s], inference fps: {:.1f} [Hz]'.format(stop - start, fps))



if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('sfa3d_ros')
    configs = parse_demo_configs()
    configs.pretrained_path = os.path.join(package_path, 'SFA3D/checkpoints/fpn_resnet_18/fpn_resnet_18_epoch_300.pth')
    model = create_model(configs)
    print('\n\n' + '-*=' * 30 + '\n\n')
    assert os.path.isfile(configs.pretrained_path), "No file at {}".format(configs.pretrained_path)
    model.load_state_dict(torch.load(configs.pretrained_path, map_location='cuda:0'))
    print('Loaded weights from {}\n'.format(configs.pretrained_path))
    configs.device = torch.device('cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx))
    model = model.to(device=configs.device)
    model.eval()

    rospy.init_node('SuperFastObjectDetection', anonymous=True)
    pub = rospy.Publisher('detected_objects', MarkerArray, queue_size=10)

    rospy.Subscriber("~input", PointCloud2, on_scan)
    rospy.spin()
