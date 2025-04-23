import tkinter as tk

WINDOW_SIZE = 600
VIRTUAL_MIN = -1.5
VIRTUAL_MAX = 1.5

def pixel_to_virtual(x, y):
    # ピクセル座標 (0〜600) を [-1.5〜+1.5] にスケーリング
    vx = VIRTUAL_MIN + (x / WINDOW_SIZE) * (VIRTUAL_MAX - VIRTUAL_MIN)
    vy = VIRTUAL_MAX - (y / WINDOW_SIZE) * (VIRTUAL_MAX - VIRTUAL_MIN)  # y軸は上下逆
    return round(vx, 3), round(vy, 3)

def on_click(event):
    vx, vy = pixel_to_virtual(event.x, event.y)
    print("{" + f"{vx}, {vy}, " "0.0" + "},")
    canvas.create_text(event.x, event.y, text=f"({vx}, {vy})", anchor="nw", fill="blue")

# ウィンドウ設定
root = tk.Tk()
root.title("Virtual Coordinate Viewer")
root.geometry(f"{WINDOW_SIZE}x{WINDOW_SIZE}")

# キャンバス作成
canvas = tk.Canvas(root, width=WINDOW_SIZE, height=WINDOW_SIZE, bg="white")
canvas.pack()

# クリックイベント登録
canvas.bind("<Button-1>", on_click)

# 実行
root.mainloop()