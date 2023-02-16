import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

class ObjectDetector:
    def __init__(self, model_path, confidence_threshold=0.5):
        self.confidence_threshold = confidence_threshold

        # Load the YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.to(torch.device('cuda' if torch.cuda.is_available() else 'cpu'))

        # Load the image conversion bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/summit_xl/front_rgbd_camera/scan', Image, self.image_callback)

    def image_callback(self, msg):
        # Convert the ROS image message to a numpy array
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform object detection on the image
        results = self.model(img)

        # Extract the bounding boxes, class labels, and confidence scores from the results
        boxes = results.xyxy[0].cpu().numpy()
        labels = results.names[results.xyxy[0][:, -1].long().cpu()]
        scores = results.xyxy[0][:, -1].cpu().numpy()

        # Filter out detections with confidence scores below the threshold
        mask = scores > self.confidence_threshold
        boxes = boxes[mask]
        labels = labels[mask]
        scores = scores[mask]

        # Print the detected objects
        for box, label, score in zip(boxes, labels, scores):
            print(f'Detected {label} with confidence {score:.2f} at position ({box[0]:.2f}, {box[1]:.2f})')

if __name__ == '__main__':
    rospy.init_node('object_detector')
    detector = ObjectDetector(model_path='yolov5s.pt', confidence_threshold=0.5)
    rospy.spin()
