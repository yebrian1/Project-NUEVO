import numpy as np
import cv2
import os
import onnxruntime as ort
from PIL import Image

# MediaPipe standard import (works since the downgrade)
import mediapipe as mp
mp_face_detection = mp.solutions.face_detection

# Enable HEIC image support for phone photos
from pillow_heif import register_heif_opener
register_heif_opener()

def preprocess_for_onnx(crop_rgb):
    """
    Manually replicates PyTorch's transforms.Compose:
    Resize -> ToTensor (Scale to 0-1 & CHW) -> Normalize
    """
    # 1. Resize to exactly 224x224
    img_resized = cv2.resize(crop_rgb, (224, 224))
    
    # 2. Convert to float32 and scale pixel values from [0, 255] to [0.0, 1.0]
    img_float = img_resized.astype(np.float32) / 255.0
    
    # 3. Apply ImageNet Normalization means and standard deviations
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img_normalized = (img_float - mean) / std
    
    # 4. PyTorch/ONNX expects Channels-First (CHW), OpenCV provides Channels-Last (HWC)
    # Transpose dimensions from (224, 224, 3) to (3, 224, 224)
    img_chw = np.transpose(img_normalized, (2, 0, 1))
    
    # 5. Add the Batch dimension so the final shape is (1, 3, 224, 224)
    input_tensor = np.expand_dims(img_chw, axis=0)
    
    return input_tensor

def numpy_softmax(x):
    """Computes softmax probabilities for an array of raw logits."""
    # Subtract max for numerical stability before calculating exponentials
    e_x = np.exp(x - np.max(x, axis=1, keepdims=True))
    return e_x / e_x.sum(axis=1, keepdims=True)

def run_onnx_scrubber(data_root):
    test_dir = os.path.join(data_root, 'zoomed_test')
    if not os.path.exists(test_dir):
        print(f"Error: Directory '{test_dir}' not found.")
        return

    valid_exts = ('.jpg', '.jpeg', '.png', '.ppm', '.bmp', '.pgm', '.tif', '.tiff', '.webp', '.heic')
    image_files = [f for f in os.listdir(test_dir) if f.lower().endswith(valid_exts)]
    
    if not image_files:
        print(f"No images found in {test_dir}.")
        return
        
    print(f"Found {len(image_files)} zoomed images. Booting ONNX pipeline...")

    # 1. Initialize ONNX Runtime Session (Pure CPU execution)
    onnx_path = os.path.join(data_root, 'mobilenet_customer_classifier.onnx')
    if not os.path.exists(onnx_path):
        print(f"Error: ONNX model not found at {onnx_path}")
        return
        
    session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name

    # 2. MediaPipe Face Detection Setup
    face_detector = mp_face_detection.FaceDetection(min_detection_confidence=0.5, model_selection=1)

    classes = ['boy', 'girl'] # Adjust based on your alphabetical folder structure
    current_idx = 0

    print("\n--- CONTROLS ---")
    print("Press 'D' -> Next Image")
    print("Press 'A' -> Previous Image")
    print("Press 'ESC' -> Exit Viewer\n")

    # 3. Interactive Loop
    while True:
        filename = image_files[current_idx]
        filepath = os.path.join(test_dir, filename)
        
        try:
            # Load raw image
            pil_img = Image.open(filepath).convert('RGB')
            raw_rgb = np.array(pil_img)
            raw_bgr = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR)
            
            # Default state variables
            prediction = "NO FACE DETECTED"
            confidence_score = 0.0
            face_crop = None
            display_frame = raw_bgr.copy()
            
            # --- STAGE 1: LOCALIZATION (MediaPipe) ---
            results = face_detector.process(raw_rgb)
            
            if results.detections:
                detection = results.detections[0]
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = raw_bgr.shape
                
                # Convert relative bounding box to pixel coordinates
                x = int(bboxC.xmin * iw)
                y = int(bboxC.ymin * ih)
                w = int(bboxC.width * iw)
                h = int(bboxC.height * ih)
                
                # Apply 20% safety padding
                pad_x = int(w * 0.2)
                pad_y = int(h * 0.2)
                
                x1 = max(0, x - pad_x)
                y1 = max(0, y - pad_y)
                x2 = min(iw, x + w + pad_x)
                y2 = min(ih, y + h + pad_y)
                
                # Slice out the cropped array
                face_crop = raw_bgr[y1:y2, x1:x2]
                
                # Draw yellow localization box on the main display frame
                box_thickness = max(2, int(iw * 0.005))
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 255), box_thickness)
                
                # --- STAGE 2: CLASSIFICATION (ONNX Runtime) ---
                if face_crop.size > 0:
                    # Convert BGR back to RGB for the model
                    crop_rgb = cv2.cvtColor(face_crop, cv2.COLOR_BGR2RGB)
                    
                    # Apply pure NumPy preprocessing
                    input_tensor = preprocess_for_onnx(crop_rgb)
                    
                    # Run the ONNX graph
                    outputs = session.run(None, {input_name: input_tensor})
                    
                    # Apply Softmax to raw output logits
                    probs = numpy_softmax(outputs[0])[0]
                    
                    # Extract highest confidence class
                    class_idx = np.argmax(probs)
                    prediction = classes[class_idx]
                    confidence_score = probs[class_idx] * 100

            # --- STAGE 3: UI RENDERING ---
            # Scale down massive phone images to fit on screen
            dh, dw, _ = display_frame.shape
            max_dim = 720
            if dh > max_dim or dw > max_dim:
                scale = max_dim / max(dh, dw)
                display_frame = cv2.resize(display_frame, (int(dw * scale), int(dh * scale)))
                
            # Draw Picture-in-Picture (PIP) of the exact crop the model saw
            if face_crop is not None and face_crop.size > 0:
                pip_size = 200
                pip = cv2.resize(face_crop, (pip_size, pip_size))
                # Add white border to PIP
                pip = cv2.copyMakeBorder(pip, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=(255, 255, 255))
                ph, pw, _ = pip.shape
                
                # Render in top right corner
                margin = 15
                if display_frame.shape[1] > pw + margin and display_frame.shape[0] > ph + margin:
                    display_frame[margin:margin+ph, display_frame.shape[1]-pw-margin:display_frame.shape[1]-margin] = pip
            
            # Render Text Overlay
            box_color = (0, 255, 0) if confidence_score > 80 else (0, 165, 255)
            if prediction == "NO FACE DETECTED":
                box_color = (0, 0, 255)
            
            overlay_width = max(450, int(display_frame.shape[1] * 0.4))
            cv2.rectangle(display_frame, (0, 0), (overlay_width, 75), (0, 0, 0), -1)
            
            overlay_text = f"Class: {prediction.upper()} ({confidence_score:.1f}%)" if prediction != "NO FACE DETECTED" else prediction
            cv2.putText(display_frame, overlay_text, (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2, cv2.LINE_AA)
            cv2.putText(display_frame, f"[{current_idx + 1}/{len(image_files)}] {filename} (ONNX Engine)", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            cv2.imshow("ONNX Rover Pipeline Viewer", display_frame)
            
        except Exception as e:
            print(f"Error rendering {filename}: {e}")
            
        # Keyboard Listeners
        key = cv2.waitKey(0) & 0xFF
        if key in [ord('d'), ord('D')]:
            current_idx = (current_idx + 1) % len(image_files)
        elif key in [ord('a'), ord('A')]:
            current_idx = (current_idx - 1) % len(image_files)
        elif key == 27:
            break
            
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_onnx_scrubber(r"D:\162E_Training")