import rospy,time,sys,cv2,copy,math,collections
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D, Twist
from skimage.measure import ransac, LineModelND, CircleModel


def get_img_ros(video):
    """
    Receives a video capture port and returns image and 
    ROS Image message.
    video: capture port
    """

    # reading image from  camera
    _,cv_image = video.read() 

    # converting from image to ROS Image message	
    bridge = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")

    return cv_image, img_msg

def get_cam(video):
    """
    Receives a video carptura port and returns an image.
    vide: capture port
    """

    # reading image from camera
    _,cv_image = video.read() 

    return cv_image

def get_centroid(cv_img, mask, put_text=False, draw_contour=False):
    """
    Finds image centroid and contourn
    cv_img: input image RGB
    mask: binary image mask
    """

    cv_output = cv_img.copy()
    
    # fiding mask contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
 
    #contours = contours[1]
 
    # contours parameters
    area = 0
    moment = []
    cont_out = []
    centroid = [0,0]


    # fiding great area in the mask
    for c in contours:
        M = cv2.moments(c)        
        if (M["m00"] > area):
            area = M["m00"]
            moment = M
            cont_out = [c]
    
    # computing centroid
    centroid[0] = int(moment["m10"]/moment["m00"])
    centroid[1] = int(moment["m01"]/moment["m00"])

    # drawning image output elements
    cv2.circle(cv_output, (centroid[0], centroid[1]), 4, (255,0,0),-1)
    if draw_contour:
        cv2.drawContours(cv_output, cont_out ,-1,(0,255,0),1)

    if put_text:
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (centroid[0],centroid[1])
        fontScale = 0.5
        fontColor = (255,255,255)
        lineType = 1
        text = '('+str(centroid[0])+', '+str(centroid[1]+10)+')'

        cv2.putText(cv_output,text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)


    return centroid, cv_output

def get_base(cv_img, mask, put_text=False):
    
    """
    Finds image base and bouding box
    cv_img: input image RGB
    mask: binary image mask
    """

    cv_output = cv_img.copy()
    
    # fiding mask contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
    

    # contours parameters
    area = 0
    cont_out = []

    print('New code')

    # fiding great area in the mask
    for c in contours:
        M = cv2.moments(c)        
        if (M["m00"] > area):
            area = M["m00"]
            cont_out = [c]
    
    contours = cont_out

    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)

    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])


    for i in range(len(contours)):
        # contour parameters
        x = boundRect[i][0]
        y = boundRect[i][1]
        w = boundRect[i][2]
        h = boundRect[i][3]

    
    high_corner_x = x
    high_corner_y = y
    low_corner_x = x+w
    low_corner_y = y+h

    # getting the center point of the base of the rectangle
    base_x = low_corner_x - (int(w/2))
    base_y = low_corner_y 
    base = [base_x,base_y]

    # drawning features
    cv2.rectangle(cv_output,
                (high_corner_x,high_corner_y),
                (low_corner_x,low_corner_y),
                (0,255,0),2)

    # drowning image output elements
    cv2.circle(cv_output, (base_x, base_y), 4, (255,0,0),-1)

    if put_text:
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (base_x,base_y+10)
        fontScale = 0.5
        fontColor = (255,255,255)
        lineType = 1
        text = '('+str(base_x)+', '+str(base_y)+')'

        cv2.putText(cv_output,text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)


    return base, cv_output


# Unused methods
'''
def get_mask_old(self,input_img,th=0.025,kernel=3,inter=4):
    
    #Creates the mask using ExGExR from the input image receiving the
    #kernel size and number of iteration to dilate algorithm
    
    # image parameters
    rows, col, channels = input_img.shape

    # Image blurring 
    input_img = cv2.GaussianBlur(input_img,(5,5),20)

    # Getting roi from image
    v_min = int(input_img.shape[1]/2) - 120
    v_max = v_min+240

    # Spliting image layers
    (B, G, R) = cv2.split(input_img)


    # Normalizing the image layer 
    R_norm = R/float(np.max(R))
    G_norm = G/float(np.max(G))
    B_norm = B/float(np.max(B))
    
    # Creating the features
    ExG = 2*G_norm - R_norm - B_norm
    ExG_av = (2*G_norm - R_norm - B_norm)/2
    ExGExR = ExG - (1.2*R_norm - G_norm)

    #normExG = np.uint8(((ExG - np.min(ExG))/(np.max(ExG) - np.min(ExG)))*255)
    #cv2.imshow('normExG channel',normExG)
    #cv2.imshow('ExGExR channel',ExGExR)
    
    cv2.imshow('ExG channel',ExG)
    cv2.imshow('ExG_av channel',ExG_av)
    
    
    # Creating the binary mask
    th_l = 0.025
    th_h = 0.05
    mask = copy.deepcopy(ExG_av)

    mask = cv2.GaussianBlur(mask,(3,3),20)


    mask[mask < th_l] = 0
    mask[mask > th_h] = 0
    mask[mask != 0] = 1



    #mask[mask < th] = 0
    #mask[mask >= th] = 1


    mask = np.uint8(mask*255)
    mask_out = np.full([rows,col],0,dtype=np.uint8)
    mask_out = mask
    mask_out[:,v_min:v_max] = mask[:,v_min:v_max]
    #mask_out[:,v_min:v_max] = np.multiply(mask_out[:,v_min:v_max],mask[:,v_min:v_max])*255
    
    #mask_out = np.uint8((mask_out[:,v_min:v_max],mask[:,v_min:v_max]) * 255)
    
    # Improving the mask with binary operations 
    kernel = kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
    #np.ones((kernel),np.uint8)
    #mask_out = cv2.dilate(mask_out,kernel,iterations=inter)
    #mask_out = cv2.erode(mask_out,kernel,iterations=inter)
    
    mask_out = cv2.morphologyEx(mask_out, cv2.MORPH_CLOSE, kernel) 
    mask_out = cv2.morphologyEx(mask_out, cv2.MORPH_OPEN, kernel) 
    mask_out = cv2.morphologyEx(mask_out, cv2.MORPH_CLOSE, kernel) 
    mask_out = cv2.morphologyEx(mask_out, cv2.MORPH_OPEN, kernel) 



    cv2.imshow('Mask out channel',mask_out)
    
    return mask_out
def get_mask_from_depth(self,img):

    gray_img = copy.copy(img)

    # Normalizing the image
    gray_img = np.uint8((gray_img /np.max(gray_img))*255)


    print(f'Original image: {img[320][240]}\nShape: {img.shape}')
    print(f"Max value: {np.max(gray_img)}\nPosition: {np.argmax(gray_img)}")
    cv2.imshow('Normalized Img',gray_img)
    edge = cv2.Canny(gray_img,100,200)
    cv2.imshow('Edge',edge)
   
    #hist = cv2.calcHist(img,[0],None,[256],[0,256])
    #gray_img = np.array(img,dtype=int)
    #gray_img = gray_img.astype(np.uint8)
    #gray_img = cv2.equalizeHist(gray_img)
    clahe = cv2.createCLAHE(clipLimit=80)
    gray_img = clahe.apply(gray_img)
    print(f'\nConverted image: {img[320,240]}')
    print(f"Max value: {np.max(gray_img)}\nPosition: {np.argmax(gray_img)}")
    cv2.imshow('Equalized Img',gray_img)
    #_,gray_img = cv2.threshold(gray_img,200,255,cv2.THRESH_BINARY)
    _,gray_img = cv2.threshold(gray_img,0,255,
                                cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #_,gray_img = cv2.threshold(gray_img, th, max_val, cv2.THRESH_TRUNC)
    

    return gray_img
'''
