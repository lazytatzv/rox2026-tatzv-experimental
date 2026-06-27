import sys
import tkinter as tk

def analyze():
    # Run tk without displaying a window
    root = tk.Tk()
    root.withdraw()
    
    img_path = 'main_ws/src/bringup/robot_bringup/models/rox2026_field/materials/textures/tag_0.png'
    img = tk.PhotoImage(file=img_path)
    w = img.width()
    h = img.height()
    
    mid_y = h // 2
    row = []
    for x in range(w):
        color = img.get(x, mid_y)
        if isinstance(color, str):
            r, g, b = map(int, color.split())
        else:
            r, g, b = color[:3]
        brightness = (r + g + b) // 3
        row.append(1 if brightness > 128 else 0)
        
    blocks = []
    current_val = row[0]
    current_len = 1
    for val in row[1:]:
        if val == current_val:
            current_len += 1
        else:
            blocks.append((current_val, current_len))
            current_val = val
            current_len = 1
    blocks.append((current_val, current_len))
    
    print(f"Image Size: {w}x{h}")
    print(f"Blocks: {blocks}")
    
    # Calculate cell sizes based on grid layout.
    # AprilTag has white border, black border, internal data, black border, white border.
    # In blocks, look for the minimum length of black/white blocks.
    lengths = [length for val, length in blocks if length > 5]
    if not lengths:
        print("Could not analyze lengths.")
        return
        
    min_len = min(lengths)
    print(f"Minimum block length: {min_len} px")
    print(f"Estimated cell count: {w / min_len}")
    
    # Try to find a cell count that matches common tag sizes:
    # 36h11 has 10x10 cells. (8x8 payload + 2x2 border)
    # 16h5 has 8x8 cells. (6x6 payload + 2x2 border)
    # Let's print out what cell size yields 10 or 8.
    for n in [8, 10]:
        cell_w = w / n
        print(f"If cell count is {n}x{n}, cell width would be {cell_w} px")

if __name__ == '__main__':
    analyze()
