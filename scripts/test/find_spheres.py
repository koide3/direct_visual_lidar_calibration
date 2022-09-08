#!/usr/bin/python3
import cv2
import numpy


class FindSpheres(object):
	def __init__(self, image_path):
		self.is_mouse_down = False
		self.mouse_down_pos = (0, 0)
		self.image = cv2.imread(image_path)

		cv2.imshow('image', self.image)
		cv2.setMouseCallback('image', self.on_mouse)

		while cv2.waitKey(10) != ord('q'):
			pass
	
	def on_mouse(self, event, x, y, flags, userdata):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.is_mouse_down = True
			self.mouse_down_pos = (x, y)
			return
		
		if event == cv2.EVENT_LBUTTONUP:
			self.is_mouse_down = False
			self.mouse_up_pos = (x, y)

			p0 = self.mouse_down_pos
			p1 = self.mouse_up_pos

			mask = numpy.zeros(self.image.shape[:2], numpy.uint8)
			bgd_model = numpy.zeros((1, 65), numpy.float64)
			fgd_model = numpy.zeros((1, 65), numpy.float64)

			tl = min(p0[0], p1[0]), min(p0[1], p1[1])
			br = max(p0[0], p1[0]), max(p0[1], p1[1])
			rect = (tl[0], tl[1], br[0] - tl[0], br[1] - tl[1])

			print(rect)

			cv2.grabCut(self.image, mask, rect, bgd_model, fgd_model, 5, cv2.GC_INIT_WITH_RECT)

			mask2 = numpy.where((mask == 2) | (mask == 0), 0, 255).astype('uint8')

			cv2.imshow('mask', mask2)
			

		if self.is_mouse_down:
			canvas = numpy.copy(self.image)
			cv2.rectangle(canvas, self.mouse_down_pos, (x, y), (0, 255, 0))
			cv2.imshow('image', canvas)


def main():
	image_path = '/home/koide/datasets/lidar_camera/ouster_reference_processed/2022-09-08-14-17-56.bag.png'
	find_spheres = FindSpheres(image_path)


if __name__ == '__main__':
	main()
