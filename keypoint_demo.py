# keypoint_demo.py
import cv2
import numpy as np
from mmpose.apis import MMPoseInferencer

def draw_keypoints(frame, keypoints, keypoint_scores, threshold=0.3):
    """Draw COCO 17-keypoint skeleton on frame."""
    
    # COCO 17 skeleton connections (joint pairs)
    SKELETON = [
        (0, 1), (0, 2),           # nose to eyes
        (1, 3), (2, 4),           # eyes to ears
        (5, 6),                   # shoulders
        (5, 7), (7, 9),           # left arm
        (6, 8), (8, 10),          # right arm
        (5, 11), (6, 12),         # shoulders to hips
        (11, 12),                 # hips
        (11, 13), (13, 15),       # left leg
        (12, 14), (14, 16),       # right leg
    ]
    
    KEYPOINT_NAMES = [
        'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
        'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
        'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
        'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
    ]
    
    h, w = frame.shape[:2]
    
    for person_kps, person_scores in zip(keypoints, keypoint_scores):
        # Draw skeleton lines
        for (j1, j2) in SKELETON:
            if person_scores[j1] > threshold and person_scores[j2] > threshold:
                pt1 = (int(person_kps[j1][0]), int(person_kps[j1][1]))
                pt2 = (int(person_kps[j2][0]), int(person_kps[j2][1]))
                # Clamp to frame bounds
                if (0 <= pt1[0] < w and 0 <= pt1[1] < h and
                    0 <= pt2[0] < w and 0 <= pt2[1] < h):
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
        
        # Draw keypoints
        for idx, (kp, score) in enumerate(zip(person_kps, person_scores)):
            if score > threshold:
                x, y = int(kp[0]), int(kp[1])
                if 0 <= x < w and 0 <= y < h:
                    # Color by confidence: green=high, yellow=medium
                    color = (0, 255, 0) if score > 0.7 else (0, 200, 200)
                    cv2.circle(frame, (x, y), 4, color, -1)
                    cv2.circle(frame, (x, y), 5, (255, 255, 255), 1)  # white border
    
    return frame


def main():
    # Initialize inferencer with RTMPose
    # det_model: person detector
    # pose2d: pose estimator
    inferencer = MMPoseInferencer(
        pose2d='human',  # uses the built-in alias — downloads everything automatically
        device='cuda:0'
    )
    
    cap = cv2.VideoCapture(0)  # 0 = default webcam; use 1, 2, etc. for others
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Press 'q' to quit, 's' to save a frame")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Run inference
        # scope='mmpose' is important to avoid namespace issues
        result_generator = inferencer(
            frame,
            show=False,           # we handle display ourselves
            return_datasamples=False
        )
        results = next(result_generator)
        
        # Extract keypoints from results
        predictions = results.get('predictions', [[]])[0]
        
        all_keypoints = []
        all_scores = []
        
        for pred in predictions:
            kps = pred.get('keypoints', [])
            scores = pred.get('keypoint_scores', [])
            if len(kps) > 0:
                all_keypoints.append(np.array(kps))
                all_scores.append(np.array(scores))
        
        # Draw on frame
        if all_keypoints:
            frame = draw_keypoints(frame, all_keypoints, all_scores)
        
        # Show FPS
        cv2.putText(frame, f'Subjects: {len(all_keypoints)}', 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        cv2.imshow('MMPose RTMPose Keypoints', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('keypoint_capture.jpg', frame)
            print("Frame saved.")
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()