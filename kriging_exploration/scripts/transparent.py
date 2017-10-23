#!/usr/bin/env python

import cv2
import numpy as np

import matplotlib as mpl
import matplotlib.cm as cm



def draw_legend(image_size, legend_size, vmin, vmax):
    layer1 = np.zeros((image_size, image_size, 4))
    
    step = (vmax - vmin)/float(legend_size)
    margin = (image_size-legend_size)/2
    
    norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
    cmap = cm.jet
    colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

    vp = range(int(np.floor(vmin)),int(np.ceil(vmax)), int(np.ceil(step)))
    print len(vp)        

    print step
    if step>1.0:
        ind = 0
        while ind < legend_size:
            a= colmap.to_rgba(int(vmin+(ind*step)))                
            b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))                
            cv2.rectangle(layer1, (int(ind+margin), int(580)), (int(ind+1+margin), int(600)), b , thickness=-1)
            print ind, ind+margin, ind+1+margin
            ind+=1
    else:
        
    

    return layer1

#layer2 = np.zeros((640, 640, 4))


res = draw_legend(640, 560, 0.0, 5000.0)

cv2.imwrite("out2.png", res)
cv2.imshow('layers', res)
cv2.waitKey(0)