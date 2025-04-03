import cv2
import numpy as np
import GUI 

# Enter sequential code!

while True:
    image = GUI.getImage()
    if image is not None:

        # change to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        GUI.showImage(gray)
        
        # morphological operations
        kernel = np.ones((5,5), np.uint8)
        dilatied_image = cv2.dilate(gray, kernel, iterations=1)
        # GUI.showImage(dilated_image)

        eroded_image = cv2.erode(gray, kernel, iterations=1)
        # GUI.showImage(eroded_image)
        
        # color filter
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        filtered_image = cv2.bitwise_and(image, image, mask=mask)
        GUI.showImage(filtered_image)
        
        # canny edges
        edges = cv2.Canny(gray, 100, 200)
        # GUI.showImage(edges)

        # blurring
        blurred_image = cv2.GaussianBlur(gray, (5,5), 0)
        # GUI.showImage(blurred_image)    

        # sharpening
        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpened_image = cv2.filter2D(gray, -1, sharpen_kernel)
        # GUI.showImage(sharpened_image)

        # optical flow
        prev_gray = gray
        next_frame = GUI.getImage()
        if next_frame is not None:
            next_gray = cv2.cvtColor(next_frame, cv2.COLOR_BGR2GRAY)
            flow = cv2.calcOpticalFlowFarneback(prev_gray, next_gray, None, 
                                              0.5, 3, 15, 3, 5, 1.2, 0)
            hsv_flow = np.zeros_like(image)
            hsv_flow[...,1] = 255
            mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
            hsv_flow[...,0] = ang * 180 / np.pi / 2
            hsv_flow[...,2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
            rgb_flow = cv2.cvtColor(hsv_flow, cv2.COLOR_HSV2BGR)
        
        # corner detection
        corners = cv2.cornerHarris(gray, 2, 3, 0.04)
        corners = cv2.dilate(corners, None)
        corner_img = image.copy()
        corner_img[corners > 0.01 * corners.max()] = [0, 0, 255]
    
        
        # hough transform
        # edges_hough = cv2.Canny(gray, 50, 150)
        # lines = cv2.HoughLines(edges_hough, 1, np.pi/180, 200)
        # hough_img = image.copy()
        # if lines is not None:
        #     for rho, theta in lines[0]:
        #         a = np.cos(theta)
        #         b = np.sin(theta)
        #         x0 = a * rho
        #         y0 = b * rho
        #         x1 = int(x0 + 1000 * (-b))
        #         y1 = int(y0 + 1000 * (a))
        #         x2 = int(x0 - 1000 * (-b))
        #         y2 = int(y0 - 1000 * (a))
        #         cv2.line(hough_img, (x1,y1), (x2,y2), (0,0,255), 2)
        


