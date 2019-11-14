import cv2
import numpy as np 

def convertToHSV(img):
    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return img2

def extractColors(hsv):
    blue_low = np.array([100, 50, 50]) #Color bounds for the blue picture
    blue_high = np.array([130, 255, 255])

    red_low = np.array([110, 50, 50])
    red_high = np.array([130, 255, 255])

    green_low = np.array([110, 50, 50])
    green_high = np.array([130, 255, 255])

    img_b = cv2.inRange(hsv, blue_low, blue_high)
    img_r = cv2.inRange(hsv, red_low, red_high)
    img_g = cv2.inRange(hsv, green_low, green_high)

    return img_b, img_g, img_r    


if __name__ == "__main__":
    #This is for my tests. I'll just call the above functions form main.py when doing the implementation.
    # img1 = cv2.imread("racquetball_red_green.jpg")
    img1 = cv2.imread("racquetball_blue_green.jpg")

    img1_hsv = convertToHSV(img1)

    img1_b, img1_g, img1_r = extractColors(img1_hsv)

    cv2.imshow("Img1", img1)
    cv2.imshow("Blue", img1_b)
    cv2.imshow("Green", img1_g)
    cv2.imshow("Red", img1_r)
    cv2.waitKey(0)