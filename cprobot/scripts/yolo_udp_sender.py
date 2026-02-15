#!/usr/bin/env python3
"""
Run YOLO on cam0 and AprilTag on cam1; send detections over UDP for ROS2 nodes.
  cam0 (YOLO only)  → 127.0.0.1:5000  (person: cx, area, conf, t)
  cam1 (AprilTag only) → 127.0.0.1:5010  (tag_id, margin, x, y, z, yaw, t)

Backends: picamera2 (system Python + python3-libcamera) or opencv (venv).
Requires: ultralytics, opencv-python, pupil-apriltags. Optional: picamera2.
"""
import argparse
import os
import socket
import sys
import time

import numpy as np

from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
from pupil_apriltags import Detector as AprilTagDetector


def run_yolo_and_draw(frame, model, conf_thresh, imgsz):
    """Run YOLO, draw person boxes on a copy. Return (vis_frame, cx_norm, area_norm, conf)."""
    H, W = frame.shape[:2]
    results = model.predict(frame, imgsz=imgsz, conf=conf_thresh, verbose=False)
    boxes = results[0].boxes
    vis = frame.copy()
    best = None
    for b in boxes:
        if int(b.cls[0]) != 0:
            continue
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis, "person", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        area = max(1.0, (x2 - x1) * (y2 - y1))
        cx = (x1 + x2) / 2.0
        cx_norm = (cx - W / 2.0) / (W / 2.0)
        area_norm = min(1.0, area / float(W * H))
        c = float(b.conf[0])
        if best is None or area > best[0]:
            best = (area, cx_norm, area_norm, c)
    if best is None:
        return (vis, None, None, None)
    return (vis, best[1], best[2], best[3])


def run_tag_and_draw(frame, detector, tag_size, fx, fy, cx, cy, tag_id_filter):
    """Run AprilTag, draw tag corners on a copy. Return (vis_frame, tag_id, margin, x, y, z, yaw)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=(fx, fy, cx, cy),
        tag_size=tag_size,
    )
    vis = frame.copy()
    best = None
    for t in tags:
        pts = np.array(t.corners, dtype=np.int32)
        cv2.polylines(vis, [pts], True, (0, 255, 255), 2)
        c = pts.mean(axis=0).astype(int)
        cv2.putText(vis, str(t.tag_id), tuple(c), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        if tag_id_filter != -1 and t.tag_id != tag_id_filter:
            continue
        if best is None or t.decision_margin > best.decision_margin:
            best = t
    if best is None:
        return (vis, None, None, None, None, None, None)
    x, y, z = best.pose_t.flatten().tolist()
    R = best.pose_R
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    return (vis, best.tag_id, best.decision_margin, x, y, z, yaw)


# ---------- camera backends ----------
def _make_picam(index, width, height):
    try:
        picam = Picamera2(index)
        config = picam.create_preview_configuration(main={"size": (width, height)})
        picam.configure(config)
        picam.start()
        return picam, None
    except Exception as e:
        return None, e


def _read_picam(picam, to_bgr):
    frame = picam.capture_array()
    if to_bgr and frame is not None:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


def _make_cv2(index, width, height):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None, RuntimeError(f"Cannot open camera {index}")
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap, None


def _read_cv2(cap):
    ok, frame = cap.read()
    return frame if ok else None


def main():
    p = argparse.ArgumentParser(description="cam0=YOLO→5000, cam1=AprilTag→5010")
    p.add_argument("--udp-host", default="127.0.0.1", help="UDP target host")
    p.add_argument("--yolo-port", type=int, default=5000, help="UDP port for YOLO (cam0)")
    p.add_argument("--tag-port", type=int, default=5010, help="UDP port for AprilTag (cam1)")
    p.add_argument("--width", type=int, default=640, help="Capture width")
    p.add_argument("--height", type=int, default=480, help="Capture height")
    p.add_argument("--model", default="yolo26s.pt", help="YOLO model")
    p.add_argument("--conf", type=float, default=0.4, help="YOLO confidence threshold")
    p.add_argument("--imgsz", type=int, default=416, help="YOLO inference size")
    p.add_argument("--tag-size", type=float, default=0.12, help="AprilTag size (m)")
    p.add_argument("--tag-id", type=int, default=-1, help="AprilTag ID filter (-1 = any)")
    p.add_argument("--fx", type=float, default=915.0, help="Camera fx for AprilTag (cam1)")
    p.add_argument("--fy", type=float, default=915.0, help="Camera fy")
    p.add_argument("--cx", type=float, default=320.0, help="Camera cx (default half width)")
    p.add_argument("--cy", type=float, default=240.0, help="Camera cy (default half height)")
    p.add_argument("--no-yolo", action="store_true", help="Disable YOLO (do not open cam0)")
    p.add_argument("--no-tag", action="store_true", help="Disable AprilTag (do not open cam1)")
    p.add_argument("--show", action="store_true", help="Show cv2 windows (cam0 YOLO, cam1 AprilTag)")
    p.add_argument("--opencv", action="store_true", help="Force OpenCV backend")
    args = p.parse_args()

    show_display = args.show or bool(os.environ.get("DISPLAY"))

    if args.no_yolo and args.no_tag:
        print("At least one of YOLO or AprilTag must be enabled", file=sys.stderr)
        sys.exit(1)

    use_picam = USE_PICAMERA2 and not args.opencv
    make_cam = _make_picam
    read_cam = (lambda c: _read_picam(c, True))
    backend = "picamera2"

    cam0 = None
    cam1 = None
    if not args.no_yolo:
        cam0, err = make_cam(0, args.width, args.height)
        if cam0 is None:
            print(f"Cannot open camera 0 (YOLO): {err}", file=sys.stderr)
            sys.exit(1)
    if not args.no_tag:
        cam1, err = make_cam(1, args.width, args.height)
        if cam1 is None:
            print(f"Cannot open camera 1 (AprilTag): {err}", file=sys.stderr)
            sys.exit(1)

    cam_cx = args.cx
    cam_cy = args.cy
    if cam_cx == 320.0 and args.width != 640:
        cam_cx = args.width / 2.0
    if cam_cy == 240.0 and args.height != 480:
        cam_cy = args.height / 2.0

    model = YOLO(args.model) if not args.no_yolo else None
    tag_detector = AprilTagDetector(families="tag36h11") if (AprilTagDetector and not args.no_tag) else None
    if not args.no_tag and tag_detector is None:
        print("AprilTag disabled: pip install pupil-apriltags", file=sys.stderr)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    y_addr = (args.udp_host, args.yolo_port)
    t_addr = (args.udp_host, args.tag_port)

    print(f"{backend}: cam0=YOLO, cam1=AprilTag", file=sys.stderr)
    if cam0 is not None:
        print(f"  YOLO (cam0) → {args.udp_host}:{args.yolo_port}", file=sys.stderr)
    if cam1 is not None:
        print(f"  Tag (cam1)  → {args.udp_host}:{args.tag_port}", file=sys.stderr)
    if show_display:
        print("  Visualize: cv2 windows (YOLO cam0, AprilTag cam1). Press 'q' in a window to quit.", file=sys.stderr)

    try:
        while True:
            if cam0 is not None:
                frame0 = read_cam(cam0)
                if frame0 is not None:
                    t = time.time()
                    vis0, cx_norm, area_norm, conf = run_yolo_and_draw(frame0, model, args.conf, args.imgsz)
                    line = f"nan,nan,nan,{t:.6f}\n" if cx_norm is None else f"{cx_norm:.4f},{area_norm:.4f},{conf:.4f},{t:.6f}\n"
                    sock.sendto(line.encode("utf-8"), y_addr)
                    if show_display:
                        try:
                            cv2.imshow("YOLO (cam0)", vis0)
                        except Exception:
                            show_display = False

            if cam1 is not None and tag_detector is not None:
                frame1 = read_cam(cam1)
                if frame1 is not None:
                    t = time.time()
                    vis1, tid, margin, x, y, z, yaw = run_tag_and_draw(frame1, tag_detector, args.tag_size, args.fx, args.fy, cam_cx, cam_cy, args.tag_id)
                    if tid is None:
                        line = f"nan,nan,nan,nan,nan,nan,{t:.6f}\n"
                    else:
                        line = f"{tid},{margin:.2f},{x:.4f},{y:.4f},{z:.4f},{yaw:.4f},{t:.6f}\n"
                    sock.sendto(line.encode("utf-8"), t_addr)
                    if show_display:
                        try:
                            cv2.imshow("AprilTag (cam1)", vis1)
                        except Exception:
                            show_display = False

            if show_display:
                try:
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                except Exception:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        if show_display:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        if use_picam:
            if cam0 is not None:
                cam0.stop()
            if cam1 is not None:
                cam1.stop()
        else:
            if cam0 is not None:
                cam0.release()
            if cam1 is not None:
                cam1.release()


if __name__ == "__main__":
    main()
