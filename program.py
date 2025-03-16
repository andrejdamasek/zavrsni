import cv2
import numpy as np

is_drawing = False
points = np.empty((0, 2), dtype=int)  

def draw(event, x, y, flags, param):
    global is_drawing, points
    if event == cv2.EVENT_LBUTTONDOWN:
        is_drawing = True
        points = np.vstack((points, [-1, -1])) 
        points = np.vstack((points, [x, y]))  
    elif event == cv2.EVENT_MOUSEMOVE and is_drawing:
        points = np.vstack((points, [x, y]))  
    elif event == cv2.EVENT_LBUTTONUP:
        is_drawing = False


print("Pritisni 'w' za početak crtanja, 'r' za pokretanje robota, 'q' za izlaz.")

while True:
    key = input("Unesi naredbu: ")
    
    if key == 'w':
        print("Započinjemo crtanje. Koristi miša za pisanje. Pritisni 'c' za kraj.")
        canvas = np.ones((500, 500, 3), dtype=np.uint8) * 255  
        cv2.namedWindow("Canvas")
        cv2.setMouseCallback("Canvas", draw)
        
        while True:
            temp_canvas = canvas.copy()
            for i in range(1, len(points)):
                if np.all(points[i - 1] == [-1, -1]) or np.all(points[i] == [-1, -1]):
                    continue  
                cv2.line(temp_canvas, tuple(points[i - 1]), tuple(points[i]), (0, 0, 0), 2)
            cv2.imshow("Canvas", temp_canvas)
            
            if cv2.waitKey(1) & 0xFF == ord('c'):
                print("Crtanje završeno.")
                break
        
        cv2.destroyAllWindows()
        
       
        processed_path = []
        threshold = 5  
        for i in range(len(points)):
            if np.all(points[i] == [-1, -1]):  
                continue
            if i == 0 or np.all(points[i - 1] == [-1, -1]):
                processed_path.append((points[i][0], points[i][1], 0))  
                processed_path.append((points[i][0], points[i][1], 1))  
            else:
                dist = np.linalg.norm(points[i] - points[i - 1])
                if dist > threshold:
                    processed_path.append((points[i][0], points[i][1], 0))  
                processed_path.append((points[i][0], points[i][1], 1))  

        print("Koordinate za robota:")
        for p in processed_path:
            print(p)
        
    elif key == 'r':
        print("Pokretanje robota.")
        break
    elif key == 'q':
        print("Izlazak iz programa.")
        break
