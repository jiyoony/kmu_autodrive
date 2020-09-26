# -*- coding: utf-8 -*-
import sys
import cv2
# ar_markers 라이브러리에서 detect_markers를 사용
from ar_markers import detect_markers

def main(argv):
	# 마커 인식할 이미지 파일 이름 표시
	print("the image file name is " + argv)
	# 이미지 파일에서 이미지 추출
	frame = cv2.imread(argv, cv2.IMREAD_UNCHANGED)
	# 이미지에서 marker 위치 추출(찾기)
	markers = detect_markers(frame)
	# 찾은 마커 개수 표시
	print("{} markers found.".format( len(markers )))
	# 마커 정보 표시
	for marker in markers:		
		# id = marker.id, 마커의 중심 위치 : marker.center 표시
		print("ID {}'s position is {}".format(marker.id, marker.center))
		# 해당 마커의 코너를 이쁘게(?) 표시. 사각형이니 4포인트(x,y)
		print("contours are :") 
		for pos in marker.contours:
			print("\t {}".format(pos))

if __name__ == '__main__':
	main("/home/nvidia/xycar_ws/src/race/src/marker_1.png")
