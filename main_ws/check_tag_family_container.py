import cv2
import numpy as np

def analyze():
    img_path = 'main_ws/src/bringup/robot_bringup/models/rox2026_field/materials/textures/tag_0.png'
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Failed to load image. Trying absolute container path...")
        img_path = '/root/lazytatzv_ws/main_ws/src/bringup/robot_bringup/models/rox2026_field/materials/textures/tag_0.png'
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        
    if img is None:
        print("Error: Could not load image inside container.")
        return
        
    h, w = img.shape
    
    mid_y = h // 2
    row = img[mid_y, :]
    
    blocks = []
    current_val = row[0]
    current_len = 1
    for val in row[1:]:
        val_bin = 1 if val > 128 else 0
        current_val_bin = 1 if current_val > 128 else 0
        if val_bin == current_val_bin:
            current_len += 1
        else:
            blocks.append((current_val_bin, current_len))
            current_val = val
            current_len = 1
    blocks.append((1 if current_val > 128 else 0, current_len))
    
    print(f"Image Size: {w}x{h}")
    print(f"Blocks (value, length): {blocks}")
    
    lengths = [length for val, length in blocks if length > 5]
    if not lengths:
        print("Could not analyze lengths.")
        return
        
    min_len = min(lengths)
    print(f"Minimum block length: {min_len} px")
    print(f"Estimated cell count: {w / min_len}")
    
    for n in [8, 9, 10, 11, 12]:
        cell_w = w / n
        print(f"If cell count is {n}x{n}, cell width would be {cell_w} px")

if __name__ == '__main__':
    analyze()
