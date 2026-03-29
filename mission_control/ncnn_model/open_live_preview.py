import cv2
from ultralytics import YOLO
import datetime
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--path", required=True)

args = parser.parse_args()

# 1. Load both models
tool_model = YOLO(args.path + "/toolsbest.pt")
elec_model = YOLO(args.path + "/electronicsbest.pt")

tool_model.to('cuda:0')
elec_model.to('cuda:0')

# --- ROI CONFIGURATION ---
ROI_X, ROI_Y = 150, 150
ROI_W, ROI_H = 300, 200

cap = cv2.VideoCapture(1)

# This list will hold detections for the CURRENT frame to be saved if 'C' is pressed
current_detections = []

def process_results(results, model_name, color, offset_x, offset_y, frame_to_draw):
    detections_in_frame = []
    for r in results:
        for box in r.boxes:
            lx1, ly1, lx2, ly2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = r.names[cls]

            gx1, gy1 = lx1 + offset_x, ly1 + offset_y
            gx2, gy2 = lx2 + offset_x, ly2 + offset_y
            center_x, center_y = (gx1 + gx2) // 2, (gy1 + gy2) // 2

            # Add to local list for saving later
            detections_in_frame.append(f"Model: {model_name} | Label: {label} | Center: ({center_x}, {center_y}) | Conf: {conf:.2f}")

            # Draw Label and Center Circle
            cv2.putText(frame_to_draw, f"{label} {conf:.2f}", (gx1, gy1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.circle(frame_to_draw, (center_x, center_y), 5, color, -1)
            
    return detections_in_frame

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    roi_frame = frame[ROI_Y : ROI_Y + ROI_H, ROI_X : ROI_X + ROI_W]

    tool_results = tool_model(roi_frame, verbose=False)
    elec_results = elec_model(roi_frame, verbose=False)

    annotated_frame = frame.copy()

    cv2.rectangle(annotated_frame, (ROI_X, ROI_Y), (ROI_X + ROI_W, ROI_Y + ROI_H), (255, 255, 255), 2)
    cv2.putText(annotated_frame, "Detection Zone (Press 'C' to Capture)", (ROI_X, ROI_Y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Clear previous frame's list and fill with new detections
    current_detections = []
    current_detections.extend(process_results(tool_results, "TOOL", (0, 255, 0), ROI_X, ROI_Y, annotated_frame))
    current_detections.extend(process_results(elec_results, "ELEC", (255, 0, 0), ROI_X, ROI_Y, annotated_frame))

    cv2.imshow("ROI Detection", annotated_frame)

    key = cv2.waitKey(1) & 0xFF
    
    # --- SAVE TO FILE LOGIC ---
    if key == ord("c"):
        if current_detections:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"detections_{timestamp}.txt"
            
            with open(filename, "w") as f:
                f.write(f"Detections captured at {timestamp}\n")
                f.write("-" * 30 + "\n")
                for item in current_detections:
                    f.write(item + "\n")
            
            print(f"Successfully saved {len(current_detections)} items to {filename}")
        else:
            print("Nothing detected to save!")

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
