#!/usr/bin/python3
# WARNING: SuperGlue is allowed to be used for non-commercial research purposes!!
#        : You must carefully check and follow its licensing condition!!
#        : https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE
import sys
import cv2
import json
import torch
import numpy
import argparse
from models.matching import Matching
from models.utils import (make_matching_plot_fast, frame2tensor)

def main():
  print('\033[93m' + '****************************************************************************************************' + '\033[0m')
  print('\033[93m' + '* WARNING: You are going to use SuperGlue that is not allowed to be used for commercial purposes!! *' + '\033[0m')
  print('\033[93m' + '****************************************************************************************************' + '\033[0m')

  parser = argparse.ArgumentParser(description='Initial guess estimation based on SuperGlue', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--data_path', help='Input data path')
  parser.add_argument('--superglue', choices={'indoor', 'outdoor'}, default='outdoor', help='SuperGlue weights')
  parser.add_argument('--max_keypoints', type=int, default=-1, help='Maximum number of keypoints detected by Superpoint' ' (\'-1\' keeps all keypoints)')
  parser.add_argument('--keypoint_threshold', type=float, default=0.05, help='SuperPoint keypoint detector confidence threshold')
  parser.add_argument('--nms_radius', type=int, default=4, help='SuperPoint Non Maximum Suppression (NMS) radius (Must be positive)')
  parser.add_argument('--sinkhorn_iterations', type=int, default=20, help='Number of Sinkhorn iterations performed by SuperGlue')
  parser.add_argument('--match_threshold', type=float, default=0.01, help='SuperGlue match threshold')
  parser.add_argument('--show_keypoints', action='store_true', help='Show the detected keypoints')
  parser.add_argument('--force_cpu', action='store_true', help='Force pytorch to run in CPU mode.')

  opt = parser.parse_args()
  print(opt)

  torch.set_grad_enabled(False)
  device = 'cuda' if torch.cuda.is_available() and not opt.force_cpu else 'cpu'

  print('Running inference on device \"{}\"'.format(device))
  config = {
    'superpoint': {
      'nms_radius': opt.nms_radius,
      'keypoint_threshold': opt.keypoint_threshold,
      'max_keypoints': opt.max_keypoints
    },
    'superglue': {
      'weights': opt.superglue,
      'sinkhorn_iterations': opt.sinkhorn_iterations,
      'match_threshold': opt.match_threshold,
    }
  }

  data_path = opt.data_path
  with open(data_path + '/calib.json', 'r') as f:
    calib_config = json.load(f)

  for bag_name in calib_config['meta']['bag_names']:
    print('processing %s' % bag_name)

    matching = Matching(config).eval().to(device)
    keys = ['keypoints', 'scores', 'descriptors']

    camera_image = cv2.imread('%s/%s.png' % (data_path, bag_name), 0)
    lidar_image = cv2.imread('%s/%s_lidar_intensities.png' % (data_path, bag_name), 0)

    camera_image_tensor = frame2tensor(camera_image, device)
    lidar_image_tensor = frame2tensor(lidar_image, device)

    last_data = matching.superpoint({'image': camera_image_tensor})
    last_data = {k+'0': last_data[k] for k in keys}
    last_data['image0'] = camera_image_tensor

    pred = matching({**last_data, 'image1': lidar_image_tensor})
    kpts0 = last_data['keypoints0'][0].cpu().numpy()
    kpts1 = pred['keypoints1'][0].cpu().numpy()
    matches = pred['matches0'][0].cpu().numpy()
    confidence = pred['matching_scores0'][0].cpu().numpy()

    result = { 'kpts0': kpts0.flatten().tolist(), 'kpts1': kpts1.flatten().tolist(), 'matches': matches.flatten().tolist(), 'confidence': confidence.flatten().tolist() }
    with open('%s/%s_matches.json' % (data_path, bag_name), 'w') as f:
      json.dump(result, f)

    if opt.show_keypoints:
      camera_canvas = cv2.cvtColor(camera_image, cv2.COLOR_GRAY2BGR)
      lidar_canvas = cv2.cvtColor(lidar_image, cv2.COLOR_GRAY2BGR)

      canvas = numpy.concatenate([camera_canvas, lidar_canvas], axis=1)
      for kp in kpts0:
        cv2.circle(canvas, (kp[0], kp[1]), 3, (255, 255, 255))
      for kp in kpts1:
        cv2.circle(canvas, (int(kp[0] + lidar_image.shape[1]), kp[1]), 3, (255, 255, 255))
    
      for i, match in enumerate(matches):
        if match < 0:
          continue
      
        kp0 = kpts0[i]
        kp1 = kpts1[match]

        cv2.line(canvas, (kp0[0], kp0[1]), (int(kp1[0] + lidar_image.shape[1]), kp1[1]), (0, 255, 0))

      print(canvas.shape, canvas.dtype)
      cv2.imshow('canvas', canvas)
      cv2.waitKey(0)


if __name__ == '__main__':
  main()
