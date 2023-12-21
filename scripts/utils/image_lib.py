#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Library for image processing using        #
# OpensCV.                                  #
#                                           #
# Changes:                                  #
#    * Updated to Python3 and OPencv 4.2.0  #
#    * Get the great area in the image to   #
#      compute centroid and base point.     #
#                                           #
# Author: Adalberto Oliveira                #
# Autonomous Vehicle - Infnet	            #
# Version: 1.22                             #
# Date: 21 mar 2021                         #
#                                           #
#############################################

import rospy,time,sys,cv2,copy,math,collections
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D, Twist
from skimage.measure import ransac, LineModelND, CircleModel


class ImageProcess:

    def __init__(self,roi=None):

        pass

        if roi is not None:
            self.roi = roi
        else:
            self.roi = [0,0]

        self.prev_rho = 0
        self.prev_theta = 0
        self.prev_slope = 0
        self.dt = 0.09


    def get_edges(self,image,rho=320,theta=0):
        
        image_copy = image.copy()
        #v_min = self.roi[0]
        #v_max = self.roi[1]

        # Getting roi from image
        v_min = int(image.shape[1]-1/2) - 100
        v_max = v_min+200

        # Converting to gray
        img = cv2.cvtColor(image_copy,cv2.COLOR_BGR2GRAY)
        img = cv2.equalizeHist(img)
        clahe = cv2.createCLAHE(clipLimit=200)
        img = clahe.apply(img)
        img = cv2.GaussianBlur(img,(3,3),5)
        #cv2.imshow("Hist",img)

        _,img = cv2.threshold(img,0,255,
                                    cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        #img = cv2.GaussianBlur(img,(3,3),5)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) 
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel) 
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) 
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel) 

        '''
        img = cv2.dilate(img,kernel)
        img = cv2.erode(img,kernel)
        img = cv2.dilate(img,kernel)
        img = cv2.erode(img,kernel)
        '''
        #cv2.imshow("Mask",img)
        

        contours, hierarchy = cv2.findContours(image=img[:,v_min:v_max], 
                                                mode=cv2.RETR_TREE, 
                                                method=cv2.CHAIN_APPROX_NONE)
        
        
        # contours parameters
        area = 1000
        moment = []
        cont_out = []
        centroid = []

        for c in contours:
            M = cv2.moments(c)        
            if (M["m00"] > area):
                #area = M["m00"]
                moment = M
                cont_out.append(c)
                #cont_out = c
                # computing centroid
                c_x = int(moment["m10"]/moment["m00"])
                c_y = int(moment["m01"]/moment["m00"])
                centroid.append([c_x,c_y])

        black_board = np.zeros(img.shape,dtype=np.uint8)

        cv2.drawContours(image=black_board[:,v_min:v_max], 
                                contours=cont_out, 
                                contourIdx=-1, 
                                color=(255), 
                                thickness=1, 
                                lineType=cv2.LINE_AA)
        
        img = cv2.Canny(black_board,50,200,apertureSize=7)
        
        lines = cv2.HoughLines(img, 1, np.pi / 180, 150, None, 0, 0)
        
        '''
        try:
            L = np.array(lines)
            TH =  np.average(L[:,:,0],axis=0)
            RHO = np.average(L[:,:,1],axis=0)
            
        except Exception as e:
            pass
        '''


        if lines is not None:
            L = np.array(lines)
            _rho =  lines[0][0][0] 
            _theta = lines[0][0][1] 
            
            #_rho =  np.average(L[:,:,0],axis=0)
            #_theta = np.average(L[:,:,1],axis=0)

            #dt = 0.8
            #rho = rho + (_rho - rho)*dt
            #theta = theta + (_theta - theta)*dt

            rho = _rho
            theta = _theta

        #print(rho,theta)
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #cv2.line(image_copy, pt1, pt2,(0,255,0), 2, cv2.LINE_AA)
        cv2.line(image_copy, pt1, pt2,(255,0,0), 2, cv2.LINE_AA)
        ref_pt1 = (0,0)
        ref_pt2 = (600,400)
        cv2.line(image_copy,ref_pt_1,ref_pt2,(255,0,0), 2, cv2.LINE_AA)

        '''
        
        #for c in centroid:
        #    cv2.circle(image_copy, (c[0]+v_min, c[1]), 4, (255,0,0),-1)

        
        
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(image_copy, pt1, pt2, (0,255,0), 1, cv2.LINE_AA)

        '''

        #cv2.imshow("Edges",img)
        #cv2.imshow("Edges",image_copy)


        return rho,theta
        

    def get_contours(self,mask,area=1000,roi=(0.4,0.6)):
        
        col = mask.shape[1]

        # Finding the contouns
        contours, _ = cv2.findContours(image=mask, 
                                        mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)

        moment = []
        cont_out = []
        centroid = []



        for c in contours:
            M = cv2.moments(c)        
            if (M["m00"] > area):
                moment = M
                cont_out.append(c)
                # computing centroid
                c_x = int(moment["m10"]/moment["m00"])
                c_y = int(moment["m01"]/moment["m00"])

                lim_l = int(col * roi[0])
                lim_r = int(col * roi[1]) 
                
                if (c_x >= lim_l) and (c_x <= lim_r):
                    centroid.append([c_x,c_y])        

        centroid = np.array(centroid)

        return cont_out, centroid

    
    def line_from_hough(self,H,img_shape=[640,480,3]):

        rho =  H[0] 
        theta = H[1] 

        # Creating the line parameters
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + img_shape[0]*(-b)), int(y0 + img_shape[0]*(a)))
        pt2 = (int(x0 - img_shape[0]*(-b)), int(y0 - img_shape[0]*(a)))
        
        line = [pt1,pt2]

        # Computing the normalized features
        norm_rho = (abs(rho) - (img_shape[1]/2))/(img_shape[1]/2)
        slope = round(math.atan2((pt2[0] - pt1[0]),(pt2[1] - pt1[1])),8)

        features = [norm_rho,slope]

        return line, features

    
    def get_mask_hsv(self,image,mask_lower=(40, 60, 0),mask_upper=(95, 255, 255)):
        
        # ESPERANDO QUE A IMAGEM ESTEJA NO FORMATO B-G-R
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, mask_lower, mask_upper) #  (20, 58, 58), (85, 255, 255) # (40, 35, 35), (80, 255, 255)
        
        #mask_green = mask_green/255

        return mask_green

    
    def get_mask(self,input_img,roi=100,th_l=0.04,th_h=0.08,kernel_size=(3,3)):
        
        '''
        Creates the mask using ExGExR from the input image 
        receiving the kernel size and number of iteration 
        to dilate algorithm
        '''
        
        # Getting the image parameters
        rows, col, channels = input_img.shape

        # Image blurring 
        input_img = cv2.GaussianBlur(input_img,(3,3),20)

        # Getting roi from image
        if roi is not None:
            v_min = int(input_img.shape[1]/2) - roi
            v_max = v_min+(2*roi)
        else:
            v_min = 0
            v_max = -1
    
        # Spliting image layers
        (B, G, R) = cv2.split(input_img)

        # Normalizing the image layer 
        R_norm = R/float(np.max(R))
        G_norm = G/float(np.max(G))
        B_norm = B/float(np.max(B))
        
        # Creating the features
        ExG_av = (2*G_norm - R_norm - B_norm)/2
         
        # Creating the binary mask
        mask = copy.deepcopy(ExG_av)
        mask = cv2.GaussianBlur(mask,(3,3),20)
        mask[mask < th_l] = 0
        mask[mask > th_h] = 0
        mask[mask != 0] = 1
        mask = np.uint8(mask*255)
        
        mask_out = np.full([rows,col],0,dtype=np.uint8)

        # Applying the ROI    
        mask_out[:,v_min:v_max] = mask[:,v_min:v_max]
        
        # Improving the mask with binary operations 
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, kernel_size)
        mask_out = cv2.morphologyEx(mask_out, cv2.MORPH_CLOSE, kernel) 
        mask_out = cv2.morphologyEx(mask_out, cv2.MORPH_OPEN, kernel) 
        

        return mask_out


    def get_line(self,image,rho=320,theta=0,show_img=False):

        """
            Receives a RGB/color and returns line paramters rho/theta
        """

        image_copy = cv2.resize(image, (640,480))

        # ROI
        v_min = self.roi[0]
        v_max = self.roi[1]
        roi = (0.02,0.09)
        
        # Converting to gray
        img = cv2.cvtColor(image_copy,cv2.COLOR_BGR2GRAY)
        
        # Applaying enhancements to the gray image 
        img = cv2.equalizeHist(img)
        clahe = cv2.createCLAHE(clipLimit=200)
        img = clahe.apply(img)
        img = cv2.GaussianBlur(img,(3,3),5)
        

        
        # Creating the line mask
        _,img = cv2.threshold(img,0,255,
                                    cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (5,5))
        for i in range(0):
            img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) 
            img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel) 
        
        

        contours,_ = self.get_contours(img[:,v_min:v_max],area=1000)

        # Creating the contours images
        black_board = np.zeros(img.shape,dtype=np.uint8)
        cv2.drawContours(image=black_board[:,v_min:v_max], 
                                contours=contours, 
                                contourIdx=-1, 
                                color=(255), 
                                thickness=1, 
                                lineType=cv2.LINE_AA)
        
        # Finding the lines in the image
        img = cv2.Canny(black_board,50,200,apertureSize=7)
        lines = cv2.HoughLines(img, 1, np.pi / 180, 150, None, 0, 0)
        
        
        # Colors
        blue = (255, 0, 0)
        green = (0,255,0)
        black = (0,0,0)
        white = (255,255,255)    
        red = (0,0,255)

        all_theta = []#np.full(5,0)
        all_rho = [] #np.full(5,320)
        const = 1000
        for i in range(10):

            if lines is not None and i <= len(lines)-1:
                rho =   lines[i][0][0] 
                theta = lines[i][0][1]

            else:
                rho = -319
                theta = np.pi

            if rho > 0:
                all_theta.append(theta)
            else:
                all_theta.append(theta-np.pi)

            all_rho.append(abs(rho))
            
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + const*(-b)), int(y0 + const*(a)))
            pt2 = (int(x0 - const*(-b)), int(y0 - const*(a)))

            cv2.line(image_copy, pt1, pt2, green, 1, cv2.LINE_AA)
            
            ref_pt1 = (int(image_copy.shape[1]/2),0)
            ref_pt2 = (int(image_copy.shape[1]/2),image_copy.shape[0]-1)



        av_rho = np.average(np.array(all_rho))
        av_theta = np.average(np.array(all_theta))

        a = math.cos(av_theta)
        b = math.sin(av_theta)
        x0 = a * av_rho
        y0 = b * av_rho
        pt1 = (int(x0 + const*(-b)), int(y0 + const*(a)))
        pt2 = (int(x0 - const*(-b)), int(y0 - const*(a)))
        print(f"Red line points: {pt1},{pt2}")
        cv2.line(image_copy, pt1, pt2, red, 2, cv2.LINE_AA)

                
        for i in range(0,480,20):
            ref_pt1 = (int(image_copy.shape[1]/2),i)
            ref_pt2 = (int(image_copy.shape[1]/2),i+10)
            cv2.line(image_copy,ref_pt1,ref_pt2,blue, 1, cv2.LINE_AA)
        


        cv2.line(image_copy,(20,20),(30,20),blue, 1, cv2.LINE_AA)
        cv2.line(image_copy,(40,20),(50,20),blue, 1, cv2.LINE_AA)
        cv2.line(image_copy,(20,40),(50,40),green, 2, cv2.LINE_AA)
    

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 2
        line_type = 2

        cv2.putText(image_copy,"Reference",(60, 25),font,
                    font_scale,blue,thickness,line_type)
        cv2.putText(image_copy,"Trace",(60, 45),font,
                    font_scale,green,thickness,line_type)
        #cv2.imshow("Lines",image_copy)

        cv_output = cv2.hconcat([cv2.resize(image,(320,240)), cv2.resize(image_copy,(320,240))])

        H = [rho, theta]

        #return rho,theta
        return cv_output, H
        

    def get_green_line(self,color_img,min_area=10,roi=(0.4,0.6)):

        # getting the mask from color image
        image_copy = copy.deepcopy(color_img)
        cv_output = copy.deepcopy(color_img)
        
        # Getting the image parameters
        rows, col, channels = color_img.shape
        black_img = np.zeros([rows, col],dtype=np.uint8)

        mask = self.get_mask(color_img,roi=None)

        #cv2.imshow("Mask",mask)
        
        contours,centroids = self.get_contours(mask,area=min_area,roi=roi)

        # drawing plant contours and center    
        for c in contours:
            cv2.drawContours(cv_output, [c], -1, (0, 255, 0), 2)
        for c in centroids:
            cv2.circle(cv_output, (c[0], c[1]), 2, (0,0,255), -1)

        # Filtering
        try:
            centroids = np.flip(centroids,axis=1)
            centroids = np.unique(centroids,axis=0)
            centroids = centroids[centroids[:,1].argsort()]    
        
        except:
            pass

        # For sigle (central) line
        trial_num = 100
        min_sample_num = 2

        if len(centroids) > 2:
            # Creating the model
            # robustly fit line using only inlier data, with RANSAC algorithm
            model_robust, inliers = ransac(centroids, LineModelND, 
                                                min_samples=min_sample_num,
                                                residual_threshold=1, 
                                                max_trials=trial_num)

            # Points for prediction
            y_i = 0
            y_f = rows-1

            try:
                # Prediction
                x_i = int(model_robust.predict(y_i)[1])
                x_f = int(model_robust.predict(y_f)[1])
            
            except:
                y_i = 0
                y_f = rows-1
                x_i = 0
                x_f = 0

        else:
            y_i = 0
            y_f = rows-1
            x_i = 0
            x_f = 0

        # Finding the distance
        a = (y_i - y_f)
        b = (x_f - x_i)
        c = x_i*y_f - x_f*y_i
        d = abs(c)/(math.sqrt(a**2+b**2))

        # Finding theta
        theta_line = round(math.atan2((x_f - x_i),(y_f - y_i)),5)
        
        # Drawing the reference line
        cv2.line(black_img, (x_i,y_i),(x_f,y_f), (255,255,255),1)
        
        #If you want to use the Hough tranformation on the black image, uncoment this lines 
        lines = cv2.HoughLines(black_img,1,np.pi/180,10)
        H = [lines[0][0][0],lines[0][0][1],theta_line]
    
        line_pts, _ = self.line_from_hough(H)
        cv2.line(cv_output, line_pts[0], line_pts[1], (0,0,0), 2, cv2.LINE_AA)
        cv2.line(cv_output, (x_i,y_i),(x_f,y_f), (0,0,255),2)

        for i in range(0,480,20):
            ref_pt1 = (int(col/2),i)
            ref_pt2 = (int(col/2),i+10)
            cv2.line(cv_output,ref_pt1,ref_pt2,(255,0,0), 1, cv2.LINE_AA)
                
        
        cv_output = cv2.hconcat([cv2.resize(color_img,(320,240)), cv2.resize(cv_output,(320,240))])

        return cv_output, H


    def get_control_pd(self,image,method="Green",lin_vel=1.,gains=[0.2,0.2]):

        if method == "Green":
            processed_img,H = self.get_green_line(image)
        if method == "Hough":
            processed_img,H = self.get_line(image,show_img=True)
        print(f"Method: {method}")
        #image parameters
        rows, col, channels = image.shape

        '''
        # Getting the reference values (normalized)
        rho = round((abs(H[0]) - (col/2))/(col/2),3)
        slope = round(H[2],3)
        theta = round(H[1],3)
        '''


        # Getting the reference values (normalized)
        rho = round((abs(H[0]) - (col/2))/(col/2),3)
        theta = round(H[1],3)

        '''
        # Applying low-pass filter
        self.prev_rho = self.prev_rho + self.dt*(rho - self.prev_rho)
        self.prev_theta = self.prev_theta + self.dt*(theta - self.prev_theta)
        self.prev_slope = self.prev_slope + self.dt*(slope - self.prev_slope)
        '''

        # Applying low-pass filter
        self.prev_rho = self.prev_rho + self.dt*(rho - self.prev_rho)
        self.prev_slope = self.prev_theta + self.dt*(theta - self.prev_theta)


        v = 0 
        omega = 0

        # Getting the control gains
        K_rho = gains[0]
        K_theta = gains[1]
        K_v = gains[2]

        print(f"Control Gais: \nRho: {K_rho}\nTheta: {K_theta}")
        
        # Computing the control error
        err_rho = 0 - self.prev_rho
        err_theta = 0 - self.prev_slope

        v = lin_vel - abs(err_rho)*K_v
        omega = -  K_rho*err_rho - K_theta*err_theta 

        # Filling the control object
        control_signal = Twist()
        control_signal.linear.x = round(v,3)
        control_signal.angular.z = round(omega,3)

        '''
        info_msg = f"\n*** Parameterized Features *** \
                \n  Rho: {rho}\
                \n  Theta: {theta}\
                \n  Line slope: {H[2]}\
                \n  Diff: {np.pi-H[1]} \
                \n\n*** Filtered Parameters Signals ***\
                \n  Rho: {self.prev_rho}\
                \n  Line slope: {self.prev_slope}\
                \n\n*** Control Signals ***\
                \n  Linear Velocity: {v}\
                \n  Angular Velocity: {omega}\n"
        '''

        info_msg = f"\n*** Parameterized Features *** \
                \n  Rho: {rho}\
                \n  Theta: {theta}\
                \n  Line slope: {H[1]}\
                \n\n*** Filtered Parameters Signals ***\
                \n  Rho: {self.prev_rho}\
                \n  Line slope: {self.prev_slope}\
                \n\n*** Control Signals ***\
                \n  Linear Velocity: {v}\
                \n  Angular Velocity: {omega}\n"
        
        #print(info_msg)

        rospy.loginfo(info_msg)
        return control_signal,processed_img

