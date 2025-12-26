import cv2
import os


#img1 = cv2.imread('color_image.jpg')
#img1 = cv2.resize(img1,(640,480))
#path = os.getcwd()

#cv2.imshow('image1',img1)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

# 리사이즈한 이미지를 흑백처리
#img1 = cv2.imread('color_image.jpg')
#img2 = cv2.imread('color_image.jpg', cv2.IMREAD_GRAYSCALE)

#cv2.imshow('image1',img1)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

src_file_name = 'resized_image.jpg'
dst1_file_name = 'resized_image_gray1.jpg'
dst2_file_name = 'resized_image_gray2.jpg'

img2 = cv2.imread('resized_image.jpg',cv2.IMREAD_GRAYSCALE)

