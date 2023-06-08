
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import yolov5
import time

# load pretrained model
model = yolov5.load('/home/estebanp/catkin_ws/src/final_challenge/training/yolov5n.pt')
path = "/home/estebanp/catkin_ws/src/final_challenge/training/images_resized"
print("imported model")

# set model parameters
model.conf = 0.25  # NMS confidence threshold
model.iou = 0.45  # NMS IoU threshold
model.agnostic = False  # NMS class-agnostic
model.multi_label = False  # NMS multiple labels per box
model.max_det = 1000  # maximum number of detections per image

for folder in os.listdir(path):
    for file in os.listdir(path + "/" + folder):


        start_time = time.time()
        # perform inference
        results = model("/home/estebanp/catkin_ws/src/final_challenge/training/images_resized/" + folder + "/" + file) # or img1, img2, ...

        # parse results
        predictions = results.pred[0]
        boxes = predictions[:, :4] # x1, y1, x2, y2
        scores = predictions[:, 4]
        categories = predictions[:, 5]

        # show detection bounding boxes on image
        results.show()

        end_time = time.time()
        print("Inference time: ", end_time-start_time)

        time.sleep(0.2)
