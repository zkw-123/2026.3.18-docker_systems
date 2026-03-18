#!/usr/bin/env python3
"""
Offline script to analyze C_sp (intensity std) and C_grad (Sobel gradient mean)
over a dataset of ultrasound images.

Usage example:
    python analyze_us_stability_stats.py \
        --img_glob "/ros2_ws/datasets/us_imgs/*.png" \
        --mask_path "/ros2_ws/datasets/us_mask.png"

如果没有 ROI mask，就不要传 --mask_path，会默认用整张图像。
"""

import argparse
import glob
import os

import cv2
import numpy as np


def compute_c_sp_and_c_grad(gray_img: np.ndarray, mask: np.ndarray | None):
    """Compute C_sp (std of intensity) and C_grad (mean Sobel gradient) in ROI.

    gray_img: 2D uint8 or float32 image
    mask: 2D uint8 or bool mask, >0 / True 表示 ROI；如果为 None，则使用整张图。
    """
    # 保证是 float32，方便后续计算
    if gray_img.dtype != np.float32:
        gray = gray_img.astype(np.float32)
    else:
        gray = gray_img

    if mask is not None:
        roi_idx = mask > 0
        # 如果 ROI 太小（几乎没有像素），返回 None
        if roi_idx.sum() < 10:
            return None, None
        roi_gray = gray[roi_idx]
    else:
        roi_gray = gray.reshape(-1)

    # 1) C_sp: intensity std in ROI
    c_sp = float(np.std(roi_gray))

    # 2) C_grad: mean Sobel gradient magnitude in ROI
    # Sobel 3x3, 水平 + 垂直
    sobelx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    grad_mag = np.sqrt(sobelx**2 + sobely**2)

    if mask is not None:
        grad_roi = grad_mag[roi_idx]
    else:
        grad_roi = grad_mag.reshape(-1)

    c_grad = float(np.mean(grad_roi))

    return c_sp, c_grad


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--img_glob",
        type=str,
        required=True,
        help='Glob pattern for ultrasound images, e.g. "/path/to/imgs/*.png"',
    )
    parser.add_argument(
        "--mask_path",
        type=str,
        default=None,
        help="Path to a single ROI mask image (same size as US images). "
             "If not provided, whole image is used as ROI.",
    )
    parser.add_argument(
        "--quantile",
        type=float,
        default=0.9,
        help="Quantile used to propose c_sp_max / c_grad_max (default: 0.9).",
    )
    args = parser.parse_args()

    img_paths = sorted(glob.glob(args.img_glob))
    if not img_paths:
        print(f"[ERROR] No images found for glob: {args.img_glob}")
        return

    print(f"Found {len(img_paths)} images.")

    # 读取 mask（如果有）
    mask = None
    if args.mask_path is not None:
        if not os.path.isfile(args.mask_path):
            print(f"[ERROR] mask_path not found: {args.mask_path}")
            return
        mask_img = cv2.imread(args.mask_path, cv2.IMREAD_GRAYSCALE)
        if mask_img is None:
            print(f"[ERROR] Failed to read mask image: {args.mask_path}")
            return
        mask = mask_img > 0
        print(f"Loaded mask from {args.mask_path}, shape={mask.shape}")

    c_sp_list = []
    c_grad_list = []

    for idx, img_path in enumerate(img_paths, start=1):
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"[WARN] Failed to read image: {img_path}, skip.")
            continue

        if mask is not None and mask.shape != img.shape:
            print(f"[WARN] Mask shape {mask.shape} != image shape {img.shape}, skip {img_path}.")
            continue

        c_sp, c_grad = compute_c_sp_and_c_grad(img, mask)
        if c_sp is None or c_grad is None:
            print(f"[WARN] ROI too small in {img_path}, skip.")
            continue

        c_sp_list.append(c_sp)
        c_grad_list.append(c_grad)

        if idx % 50 == 0:
            print(f"Processed {idx} images...")

    if not c_sp_list:
        print("[ERROR] No valid C_sp / C_grad samples collected.")
        return

    c_sp_arr = np.asarray(c_sp_list)
    c_grad_arr = np.asarray(c_grad_list)

    q = args.quantile

    def stats(arr, name: str):
        print(f"\n=== {name} statistics ===")
        print(f"  count       : {len(arr)}")
        print(f"  min         : {arr.min():.4f}")
        print(f"  max         : {arr.max():.4f}")
        print(f"  mean        : {arr.mean():.4f}")
        print(f"  {int(q*100)}% quantile : {np.quantile(arr, q):.4f}")

    stats(c_sp_arr, "C_sp (intensity std)")
    stats(c_grad_arr, "C_grad (Sobel mean magnitude)")

    # 推荐的上限：略高于 90% 分位数，比如乘 1.1
    q_sp = float(np.quantile(c_sp_arr, q))
    q_grad = float(np.quantile(c_grad_arr, q))

    c_sp_max_recommended = q_sp * 1.1
    c_grad_max_recommended = q_grad * 1.1

    print("\n=== Recommended normalization upper bounds ===")
    print(f"  c_sp_max   ≈ {c_sp_max_recommended:.4f}")
    print(f"  c_grad_max ≈ {c_grad_max_recommended:.4f}")
    print("You can use these values as:")
    print("  c_sp_norm   = clip(c_sp / c_sp_max,   0, 1)")
    print("  c_grad_norm = clip(c_grad / c_grad_max, 0, 1)")


if __name__ == "__main__":
    main()
