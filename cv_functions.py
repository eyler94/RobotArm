import cv2
import numpy as np 

def convertToHSV(img):
    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return img2

def extractColors(hsv):
    blue_low = np.array([100, 50, 50]) #Color bounds for the blue picture
    blue_high = np.array([130, 255, 255])

    red_low = np.array([10, 10, 10])
    red_high = np.array([150, 255, 255])

    green_low = np.array([60, 50, 50])
    green_high = np.array([90, 255, 255])

    img_b = cv2.inRange(hsv, blue_low, blue_high)
    img_r = cv2.inRange(hsv, red_low, red_high)
    img_r = cv2.bitwise_not(img_r)
    img_g = cv2.inRange(hsv, green_low, green_high)

    return img_b, img_g, img_r    

def filterNoise(img1_b, img1_g, img1_r):
    kernel = np.ones((5,5), np.uint8)
    iters = 3
    img1_b = cv2.erode(img1_b, kernel, iterations=iters)
    img1_g = cv2.erode(img1_g, kernel, iterations=iters)
    img1_r = cv2.erode(img1_r, kernel, iterations=iters)

    img1_b = cv2.dilate(img1_b, kernel, iterations=iters)
    img1_g = cv2.dilate(img1_g, kernel, iterations=iters)
    img1_r = cv2.dilate(img1_r, kernel, iterations=iters)

    return img1_b, img1_g, img1_r

def findTomatoes(img1_b, img1_g, img1_r):
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = False
    params.minArea = 100 #This can be tuned
    params.filterByCircularity = False 
    params.filterByInertia = False 
    params.filterByConvexity = False
    params.minThreshold = 100
    params.maxThreshold = 255 
    params.filterByColor = True
    params.blobColor = 255

    detector = cv2.SimpleBlobDetector_create(params)
    pts_b = detector.detect(img1_b)
    pts_g = detector.detect(img1_g)
    pts_r = detector.detect(img1_r)

    return pts_b, pts_g, pts_r

def getMinDist(pts):
    min_dist = 1e6
    target = None
    for keypt in pts:
        pt = np.array(keypt.pt)
        dist = np.linalg.norm(pt)
        if dist < min_dist:
            target = pt
    
    return target

def getColorAndTarget(pts_b, pts_g, pts_r):
    min_dist = 1e6
    target = None
    if not len(pts_r) == 0:
        color = 1
        target  = getMinDist(pts_r)
    elif not len(pts_g) == 0:
        color = 2
        target = getMinDist(pts_g)
    elif not len(pts_b) == 0:
        color = 3
        target = getMinDist(pts_b)
    else:
        color = 4
    
    return color, target

def getDistToCenter(center_pt, ball_pos): # Used for visual servoing
    ds = center_pt - ball_pos
    return ds[0], ds[1]  

if __name__ == "__main__":
    #This is for my tests. I'll just call the above functions form main.py when doing the implementation.
    img1 = cv2.imread("racquetball_red_green.jpg")
    # img1 = cv2.imread("racquetball_blue_green.jpg")

    img1_hsv = convertToHSV(img1)

    img1_b, img1_g, img1_r = extractColors(img1_hsv)
    img1_b, img1_g, img1_r = filterNoise(img1_b, img1_g, img1_r)

    #Detect individual balls
    blue_pts, green_pts, red_pts = findTomatoes(img1_b, img1_g, img1_r) #For some reason didn't work on balls that were touching. Try to keep balls farther apart
    print(red_pts) # access pts via  blue_pts[#].pt
    print(green_pts)
    print(blue_pts)

    # get color target: red = 1, green = 2, blue = 3, no targets = 4
    color, target = getColorAndTarget(blue_pts, green_pts, red_pts)
    print(color)
    print(target)

    img_size = img1_b.shape
    center_pt = np.array(img_size)/2.0 #  Will use this in visual servoing to determine if we are above the ball
    print(center_pt)

    dx, dy = getDistToCenter(center_pt, target)
    print(dx)
    print(dy)
    
    #Draw points on each picture
    img1_b = cv2.drawKeypoints(img1_b, blue_pts, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img1_g = cv2.drawKeypoints(img1_g, green_pts, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img1_r = cv2.drawKeypoints(img1_r, red_pts, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv2.imshow("Img1", img1)
    cv2.imshow("Blue", img1_b)
    cv2.imshow("Green", img1_g)
    cv2.imshow("Red", img1_r)
    cv2.waitKey(0)