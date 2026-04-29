import argparse
import os
from typing import List, Tuple

import cv2
import numpy as np


class PolygonMaskDrawer:
    def __init__(self, image_path: str, output_path: str):
        self.image_path = image_path
        self.output_path = output_path

        self.img = cv2.imread(self.image_path)
        if self.img is None:
            raise RuntimeError(f"Failed to read image: {self.image_path}")

        # 显示用副本
        self.display_img = self.img.copy()

        # 存放多边形顶点 (x, y)
        self.points: List[Tuple[int, int]] = []

        # 窗口名
        self.win_name = "Draw ROI Mask (left-click: add point, right-click: close; r: reset, s: save)"

        # mask（灰度 0/255）
        h, w = self.img.shape[:2]
        self.mask = np.zeros((h, w), dtype=np.uint8)

    def mouse_callback(self, event, x, y, flags, param):
        # 左键：添加一个点
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            self._redraw()

        # 右键：闭合多边形
        elif event == cv2.EVENT_RBUTTONDOWN:
            if len(self.points) >= 3:
                self._close_polygon()
                self._redraw(final=True)

    def _redraw(self, final: bool = False):
        """根据当前点重画显示图像"""
        self.display_img = self.img.copy()

        # 画已有的点和边
        for i, p in enumerate(self.points):
            cv2.circle(self.display_img, p, 3, (0, 0, 255), -1)
            if i > 0:
                cv2.line(self.display_img, self.points[i - 1], p, (0, 255, 0), 1)

        # 如果 final=True，说明已经闭合多边形，填充 mask 并画实心多边形轮廓
        if final and len(self.points) >= 3:
            pts = np.array(self.points, dtype=np.int32)
            cv2.fillPoly(self.display_img, [pts], (0, 255, 0))  # 可视化填满
            # mask 里也填充
            self.mask[:] = 0
            cv2.fillPoly(self.mask, [pts], 255)

    def _close_polygon(self):
        """闭合多边形：在显示上连最后一点和第一点"""
        if len(self.points) >= 3:
            cv2.line(self.display_img, self.points[-1], self.points[0], (0, 255, 0), 1)

    def reset(self):
        """重置当前多边形和 mask"""
        self.points = []
        self.display_img = self.img.copy()
        self.mask[:] = 0

    def save_mask(self):
        # 确保至少有一个有效多边形
        if len(self.points) < 3 or np.count_nonzero(self.mask) == 0:
            print("No valid polygon/mask to save. Please close a polygon first (right-click).")
            return

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        cv2.imwrite(self.output_path, self.mask)
        print(f"Mask saved to: {self.output_path}")

    def run(self):
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win_name, self.mouse_callback)

        print("操作说明：")
        print("  左键：依次添加多边形顶点")
        print("  右键：闭合多边形并生成 mask")
        print("  r 键：重置重新画")
        print("  s 键：保存 mask 并退出")
        print("  ESC：不保存退出")

        while True:
            cv2.imshow(self.win_name, self.display_img)
            key = cv2.waitKey(20) & 0xFF

            if key == 27:  # ESC
                print("退出，不保存。")
                break

            elif key in (ord('r'), ord('R')):
                print("重置 ROI。")
                self.reset()

            elif key in (ord('s'), ord('S')):
                self._redraw(final=True)  # 再确保 mask 更新
                self.save_mask()
                break

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Draw polygon ROI on an ultrasound image and save binary mask."
    )
    parser.add_argument(
        "--image",
        type=str,
        required=True,
        help="Path to an ultrasound image (png/jpg)."
    )
    parser.add_argument(
        "--output",
        type=str,
        default="us_mask.png",
        help="Path to output mask image (png, grayscale 0/255)."
    )

    args = parser.parse_args()

    drawer = PolygonMaskDrawer(args.image, args.output)
    drawer.run()


if __name__ == "__main__":
    main()
