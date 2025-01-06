from ultralytics.models.fastsam import FastSAMPrompt
import numpy as np
import cv2
import pickle
import torch
import clip
from PIL import Image
from ultralytics import YOLOWorld
from ultralytics import FastSAM
import matplotlib.pyplot as plt
import matplotlib.patches as patches

device="cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

def target_present(rgb,detector,taget_class='cloth',confidence=0.01):
    detector.set_classes([taget_class])
    detections=detector.predict(rgb,conf=confidence)
    detected_classes=detections[0].boxes.cls.tolist()
    if len(detected_classes)==0:
        return False
    
    return True

def get_target_position(rgb:np.ndarray,depth,camera,base_to_camera_tf,detector:YOLOWorld,segmenter: FastSAM,target_class='cloth',confidence=0.01,device='cpu', vis=True):
    #test

    
    detector.set_classes([target_class])
    original_size=rgb.shape[:2]
    modified_size=(original_size[0]-original_size[0]%32,original_size[1]-original_size[1]%32)


    detections = detector.predict(rgb[:modified_size[0], :modified_size[1], :], imgsz=modified_size, conf=confidence)
    detected_clases=detections[0].boxes.cls.tolist()
    detected_boxes=detections[0].boxes.xyxy.tolist()
    target_index=0

    if len(detected_clases)==0:
        return 'fail'
    
    everything_results = segmenter(detections[0].orig_img, device=device, imgsz=modified_size)
    prompt_process = FastSAMPrompt(detections[0].orig_img, everything_results, device=device)
    segments = prompt_process.box_prompt(bbox=detected_boxes[target_index])
    

    target_segment=segments[0].masks.data[0]
    mask=np.array(1-target_segment.cpu().detach().numpy())

    x = np.ma.masked_equal(np.array(depth[:modified_size[0], :modified_size[1]] / 1000), 0.0)
    y = ((camera['cy'] - np.repeat(range(modified_size[0]), modified_size[1]).reshape(modified_size)) * x) / camera['fy']
    z = ((np.tile(range(modified_size[1]), modified_size[0]).reshape(modified_size) - camera['cx']) * x) / camera['fx']


    mask_x = np.ma.masked_array(x, mask=mask)
    mask_y = np.ma.masked_array(y, mask=mask)
    mask_z = np.ma.masked_array(z, mask=mask)

    mean_x = np.mean(mask_x)
    mean_y = np.mean(mask_y)
    mean_z = np.mean(mask_z)

    q = base_to_camera_tf.transform.rotation
    t = base_to_camera_tf.transform.translation
    transform_matrix = np.array(
        [
            [1-2*q.y**2-2*q.z**2, 2*q.x*q.y-2*q.z*q.w, 2*q.x*q.z+2*q.y*q.w, t.x],
            [2*q.x*q.y+2*q.z*q.w, 1-2*q.x**2-2*q.z**2, 2*q.y*q.z-2*q.x*q.w, t.y],
            [2*q.x*q.z-2*q.y*q.w, 2*q.y*q.z+2*q.x*q.w, 1-2*q.x**2-2*q.y**2, t.z],
            [0, 0, 0, 1]
        ]
    )

    center_wrt_base = np.dot(transform_matrix, np.array([mean_x, mean_y, mean_z, 1]))[:3]

    # print('\n\n\n point', center_wrt_base)

    return center_wrt_base


def detect_clean_dirty_clothes(rgb_image,confidence=0.5):
    image = preprocess(Image.fromarray(rgb_image)).unsqueeze(0).to(device)
    # The overview has more priority than the single object
    text = clip.tokenize(['clean clothes', 'dirty clothes']).to(device)

    with torch.no_grad():
        image_features = model.encode_image(image)
        text_features = model.encode_text(text)
        
        logits_per_image, logits_per_text = model(image, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()

    print("Label probs:", probs)  # prints: [[0.9927937  0.00421068 0.00299572]]

    if probs[0][0]>0.5:
        return 'clean'
    else:
        return 'dirty'
   


   
                       
