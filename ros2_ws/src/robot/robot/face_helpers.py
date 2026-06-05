from __future__ import annotations
import os
import time
import cv2
import numpy as np
import onnxruntime as ort
import mediapipe as mp

from sensor_msgs.msg import Image
from robot.robot import Robot

# ---------------------------------------------------------------------------
# Internal Constants
# ---------------------------------------------------------------------------
MODEL_PATH = "/ros2_ws/src/robot/robot/models/mobilenet_customer_classifier.onnx"
CLASSES = ['boy', 'girl']

# ---------------------------------------------------------------------------
# Internal Preprocessing Functions
# ---------------------------------------------------------------------------
def _preprocess(crop_rgb: np.ndarray) -> np.ndarray:
    """Standard MobileNet preprocessing: Resize -> Normalize -> CHW."""
    img = cv2.resize(crop_rgb, (224, 224))
    img = img.astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img = (img - mean) / std
    return np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)

def _softmax(x: np.ndarray) -> np.ndarray:
    e_x = np.exp(x - np.max(x, axis=1, keepdims=True))
    return e_x / e_x.sum(axis=1, keepdims=True)


class CustomerClassifier:
    """
    Manages the face detection and gender classification pipeline.
    Initialize once in your main code to keep the model 'warm'.
    """
    def __init__(self, robot: Robot):
        self._robot = robot
        self._latest_frame = None
        
        # 1. Load ONNX Model
        if not os.path.exists(MODEL_PATH):
            print(f"[FaceHelper] ERROR: Model not found at {MODEL_PATH}")
            self._session = None
        else:
            print(f"[FaceHelper] Loading model: {os.path.basename(MODEL_PATH)}")
            self._session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
            self._input_name = self._session.get_inputs()[0].name

        # 2. Setup Face Detector
        self._mp_face = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=0.5, 
            model_selection=0
        )

        # 3. Background Image Subscriber
        # We subscribe to the raw stream; vision_node stays in standby (low heat).
        self._robot._node.create_subscription(
            Image, 
            "/vision/image_raw", 
            self._on_image, 
            10
        )
        print("[FaceHelper] Pipeline Initialized. Ready for get_gender() calls.")

    def _on_image(self, msg: Image):
        """Callback to update the latest camera frame."""
        # Convert raw ROS bytes to numpy array
        self._latest_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

    def get_gender(self, wait_for_face: float = 0.0) -> str | None:
        """
        Attempts to detect and classify a face in the current view.
        Returns: 'BOY', 'GIRL', or None if no face is found.
        
        If wait_for_face > 0, it will loop for that many seconds until a face is seen.
        """
        start_time = time.time()
        
        while True:
            frame = self._latest_frame
            if frame is None:
                # No images arriving yet
                if time.time() - start_time > 2.0: # 2s hard timeout for stream
                    print("[FaceHelper] WARN: No camera stream detected.")
                    return None
                time.sleep(0.1)
                continue

            # --- Stage 1: Detect Face ---
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self._mp_face.process(frame_rgb)

            if not results.detections:
                # Check if we should keep trying
                if (time.time() - start_time) < wait_for_face:
                    time.sleep(0.05)
                    continue
                return None

            # --- Stage 2: Process Best Face ---
            det = results.detections[0]
            bbox = det.location_data.relative_bounding_box
            ih, iw = frame_rgb.shape[:2]
            
            # Pixel coords with 20% padding
            x, y = int(bbox.xmin * iw), int(bbox.ymin * ih)
            w, h = int(bbox.width * iw), int(bbox.height * ih)
            px, py = int(w * 0.2), int(h * 0.2)
            
            x1, y1 = max(0, x - px), max(0, y - py)
            x2, y2 = min(iw, x + w + px), min(ih, y + h + py)
            
            face_crop = frame_rgb[y1:y2, x1:x2]
            if face_crop.size == 0 or self._session is None:
                return None

            # --- Stage 3: Classify ---
            blob = _preprocess(face_crop)
            outputs = self._session.run(None, {self._input_name: blob})
            probs = _softmax(outputs[0])[0]
            idx = np.argmax(probs)
            
            result = CLASSES[idx].upper()
            confidence = probs[idx] * 100
            
            print(f"[FaceHelper] Detection: {result} ({confidence:.1f}%)")
            return result
