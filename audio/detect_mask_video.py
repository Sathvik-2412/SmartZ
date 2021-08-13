from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import imutils
import time
import cv2
import os
from playsound import playsound
import threading
from tkinter import *
from PIL import ImageTk, Image
root = Tk()
root.title("SmartZ")
root.resizable(False,False)
#root.configure()#

# Create a frame
app = Canvas(root,)
app.grid(row=0)
buts=Frame(root)
audio=Frame(root)
#app.bind('<configure>',resize_i)
#def resize_i(event):
 #   w=event.width
  #  h=event.height
   # app.config(width=w,height=h)
    #app.scale("all",0,0,w,h)




# Create a label in the frame
lmain = Label(app)
cmain = Label(app,text="Video feed (with mask detection)",font=("San Francisco",20))
cmain.grid(row=0,column=1,pady=30)
lmain.grid(row=1,column=1,padx=10,pady=(0,10))
l1=Label(buts,text="Open the door",font=("San Francisco",15))
b1 = Button(buts,text="open",font=("San Francisco",13))
b2=Button(buts,text="close",font=("San Francisco",13))
buts.grid(row=1,padx=10,pady=10)
l1.pack(side=LEFT)
b1.pack(side=LEFT)
b2.pack(side=LEFT)

def ppyfmo():
    playsound('/Users/sathvikc/Desktop/Minor_Project/audio/please_put_your_facemask_on.m4a')

def psa():
    playsound('/Users/sathvikc/Desktop/Minor_Project/audio/please_step_aside.m4a')

def ytih():
    playsound('/Users/sathvikc/Desktop/Minor_Project/audio/your_temp_is_high.m4a')
l3=Label(audio,text="",font=("San Francisco",3))
l4=Label(audio,text="",font=("San Francisco",3))
l5=Label(audio,text="",font=("San Francisco",3))
l2=Label(audio,text="Play audio message:",font=("San Francisco",20))
a1 = Button(audio,text="PLEASE PUT YOUR FACEMASK ON!",command=ppyfmo,font=("San Francisco",13))
a2 = Button(audio,text="PLEASE STEP ASIDE!",command=psa,font=("San Francisco",13))
a3 = Button(audio,text="YOUR TEMPERATURE IS HIGH!",command=ytih,font=("San Francisco",13))

a1.config(height=5,width=25)
a2.config(height=5,width=25)
a3.config(height=5,width=25)
audio.grid(row=0,column=2,pady=10,padx=10)
l2.pack()
l3.pack()
a1.pack()
#l4.pack()
a2.pack()
#l5.pack()
a3.pack()

mssg_ppyfmo="/Users/sathvikc/Desktop/Minor_Project/audio/please_put_your_facemask_on.m4a"
mssg_psa="/Users/sathvikc/Desktop/Minor_Project/audio/please_step_aside.m4a"
mssg_ytih="/Users/sathvikc/Desktop/Minor_Project/audio/your_temp_is_high.m4a"
mssg=mssg_ppyfmo


def video_stream(frame):
    
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)


def detect_and_predict_mask(frame, faceNet, maskNet):
    
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (224, 224),
        (104.0, 177.0, 123.0))

    faceNet.setInput(blob)
    detections = faceNet.forward()
    faces = []
    locs = []
    preds = []

    for i in range(0, detections.shape[2]):
        
        confidence = detections[0, 0, i, 2]

        
        if confidence > 0.5:
            
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            (startX, startY) = (max(0, startX), max(0, startY))
            (endX, endY) = (min(w - 1, endX), min(h - 1, endY))

           
            face = frame[startY:endY, startX:endX]
            face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
            face = cv2.resize(face, (224, 224))
            face = img_to_array(face)
            face = preprocess_input(face)

            
            faces.append(face)
            locs.append((startX, startY, endX, endY))

    if len(faces) > 0:
        
        faces = np.array(faces, dtype="float32")
        preds = maskNet.predict(faces, batch_size=32)

    
    return (locs, preds)

prototxtPath = "/Users/sathvikc/Desktop/Minor_Project/fmd/face_detector/deploy.prototxt"
weightsPath = "/Users/sathvikc/Desktop/Minor_Project/fmd/face_detector/res10_300x300_ssd_iter_140000.caffemodel"
faceNet = cv2.dnn.readNet(prototxtPath, weightsPath)

maskNet = load_model("/Users/sathvikc/Desktop/Minor_Project/fmd/mask_detector.model")

print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
a=0;

while True:
    frame = vs.read()
    frame = imutils.resize(frame, height=1000,width=1000)

    
    (locs, preds) = detect_and_predict_mask(frame, faceNet, maskNet)

    
    for (box, pred) in zip(locs, preds):
        (startX, startY, endX, endY) = box
        (mask, withoutMask) = pred

        
        label = "Mask" if mask > withoutMask else "No Mask"
        color = (0, 255, 0) if label == "Mask" else (0, 0, 255)
        if a==75:
            mssg=mssg_psa
        if (label=="No Mask"):
            if(a%25==0):
                playsound(mssg)

            a=a+1

            

        else:
            a=0
            mssg=mssg_ppyfmo

        label = "{}: {:.2f}%".format(label, max(mask, withoutMask) * 100)

        
        cv2.putText(frame, label, (startX, startY - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)

    video_stream(frame)
    root.update_idletasks()
    root.update()

    if cv2.waitKey(1) & 0xFF == ord("q"):
    	break


cv2.destroyAllWindows()
vs.stop()
