# detector_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Quaternion
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_processor')
        self.bridge = CvBridge()
        self.model = YOLO('yolo11m.pt')  # Load YOLO11 medium model
        
        # Subscribe to RealSense color images
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        # Publisher for annotated images
        self.image_pub = self.create_publisher(Image, '/detections_image', 10)
        
        # Publisher for detection results
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10)

        self.get_logger().info("YOLO Detector Node Initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model.predict(cv_image, conf=0.5)
            
            annotated_image = cv_image.copy()
            detection_array = Detection2DArray()
            detection_array.header = msg.header


            # Process results
            for result in results:
                for box in result.boxes:
                    # Get bounding box coordinates
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls_id = int(box.cls[0].cpu().numpy())
                    
                    # Create Detection2D message
                    detection = Detection2D()
                    detection.bbox = BoundingBox2D()
                    detection.bbox.center.position.x = (xyxy[0] + xyxy[2]) / 2
                    detection.bbox.center.position.y = (xyxy[1] + xyxy[3]) / 2
                    detection.bbox.size_x = float(xyxy[2] - xyxy[0])
                    detection.bbox.size_y = float(xyxy[3] - xyxy[1])

                    hypothesis_pose = ObjectHypothesisWithPose()                    
                    hypothesis_pose.hypothesis.class_id = str(cls_id)  # Class name/ID as string
                    hypothesis_pose.hypothesis.score = float(conf)     # Confidence score
                    
                    hypothesis_pose.pose.pose.position = Point(
                        x=0.0,  # Not used in 2D detection
                        y=0.0,  # Not used in 2D detection
                        z=0.0   # Required but unused
                    )

                    # Default orientation (identity quaternion)
                    hypothesis_pose.pose.pose.orientation = Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0
                    )

                    # Initialize covariance to zeros (optional but recommended)
                    hypothesis_pose.pose.covariance = [0.0] * 36
    
                    detection_array.detections.append(detection)
                    detection.results.append(hypothesis_pose)  # Add to detection results

                    x1 = int(xyxy[0].item())
                    y1 = int(xyxy[1].item())
                    x2 = int(xyxy[2].item())
                    y2 = int(xyxy[3].item())
                    
                    # Draw a bounding rectangle on the object detected
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (36, 255, 12), 2)

                    # Get class name and confidence
                    class_name = self.model.names[cls_id]
                    confidence = float(conf)
                    label = f"{class_name} {confidence:.2f}"
                    
                    # Put text above the bounding box
                    cv2.putText(annotated_image, label, (x1, y1 - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
                    
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            annotated_msg.header = msg.header  # Critical for RViz alignment
            self.image_pub.publish(annotated_msg)

            # Publish detections
            detection_array.header.stamp = self.get_clock().now().to_msg()
            self.detection_pub.publish(detection_array)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()