#!/usr/bin/env python3
"""
Detection + Tracking script using YOLOv8 with DeepSORT/ByteTrack backends.
Provides real-time multi-object tracking for drone applications.
"""
import argparse
import cv2
import yaml
import time
from pathlib import Path

try:
    from ultralytics import YOLO
except ImportError:
    print("ERROR: ultralytics not installed. pip install ultralytics")
    exit(1)

# Optional trackers
try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
    HAS_DEEPSORT = True
except Exception:
    HAS_DEEPSORT = False

try:
    import bytetrack
    HAS_BYTETRACK = False  # Placeholder; integrate actual ByteTrack lib if available
except Exception:
    HAS_BYTETRACK = False


def load_config(path: str):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def create_tracker(cfg):
    backend = cfg.get('tracker', {}).get('backend', 'deepsort').lower()
    if backend == 'deepsort' and HAS_DEEPSORT:
        return DeepSort(max_age=30, n_init=3, nms_max_overlap=0.7)
    elif backend == 'bytetrack' and HAS_BYTETRACK:
        # TODO: instantiate real ByteTrack
        return None
    else:
        print(f"WARNING: Tracker backend '{backend}' unavailable, running detection only")
        return None


def draw_tracks(frame, tracks):
    for t in tracks:
        tid = t[1]
        l, t_y, r, b = map(int, t[2])
        cv2.rectangle(frame, (l, t_y), (r, b), (255, 0, 0), 2)
        cv2.putText(frame, f"ID {tid}", (l, t_y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config/detector.yaml')
    parser.add_argument('--source', default=0, help='Camera index or video path')
    args = parser.parse_args()

    cfg = load_config(args.config)
    model = YOLO(cfg['model']['weights'])
    tracker = create_tracker(cfg)

    cap = cv2.VideoCapture(0 if str(args.source).isdigit() else args.source)
    if not cap.isOpened():
        print('ERROR: could not open source')
        return 1

    print('Starting detection+tracking. Press q to quit')
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        results = model.predict(frame, conf=cfg['model'].get('confidence_threshold', 0.25), verbose=False)
        detections = []
        for r in results:
            for b in r.boxes:
                x1, y1, x2, y2 = map(float, b.xyxy[0].cpu().numpy())
                conf = float(b.conf[0])
                cls = int(b.cls[0])
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'conf': conf,
                    'cls': cls
                })
        tracks_draw = []
        if tracker and detections:
            # DeepSort expects [ [x1,y1,x2,y2,confidence, class] ]
            ds_inp = [[d['bbox'][0], d['bbox'][1], d['bbox'][2], d['bbox'][3], d['conf'], d['cls']] for d in detections]
            outputs = tracker.update_tracks(ds_inp, frame=frame)
            for tr in outputs:
                if not tr.is_confirmed():
                    continue
                l, t_y, r, b = tr.to_ltrb()
                tracks_draw.append((tr, tr.track_id, (l, t_y, r, b)))
        # Draw detections
        for d in detections:
            x1, y1, x2, y2 = map(int, d['bbox'])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        frame = draw_tracks(frame, tracks_draw)
        cv2.imshow('Track', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
