import cv2
import numpy as np
import matplotlib.pyplot as plt

is_drawing = False
points = np.empty((0, 2), dtype=int)  


def save_points_txt(filename="points.txt"):
    with open(filename, "w") as file:
        for point in points:
            file.write(f"{point[0]} {point[1]}\n")
    print(f"Podaci spremljeni u {filename}")


def load_points_txt(filename="points.txt"):
    global points
    try:
        with open(filename, "r") as file:
            points = np.array([[int(x), int(y)] for x, y in (line.split() for line in file)], dtype=int)
        print(f"Podaci učitani iz {filename}")
    except FileNotFoundError:
        print("Datoteka ne postoji.")


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


def draw_from_points():
    if len(points) == 0:
        print("Nema podataka za crtanje.")
        return
    
    plt.figure(figsize=(5, 5), dpi=100)
    x_vals, y_vals = [], []
    for i in range(1, len(points)):
        if np.all(points[i - 1] == [-1, -1]) or np.all(points[i] == [-1, -1]):
            x_vals.append(None)
            y_vals.append(None)
        else:
            x_vals.append(points[i][0])
            y_vals.append(points[i][1])
    
    plt.plot(x_vals, y_vals)
    plt.gca().invert_yaxis()
    plt.title("Učitani crtež")
    plt.show()


while True:
    
    print("Pritisni 'w' za početak crtanja, 'r' za pokretanje robota, 's' za spremanje, 'l' za učitavanje, 'b' za brisanje ,'d' za crtanje iz datoteke, 'q' za izlaz.")
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
    
    elif key == 's':
        save_points_txt()
    
    elif key == 'l':
        load_points_txt()
    
    elif key == 'd':
        draw_from_points()
    
    elif key == 'r':
        print("Pokretanje robota.")
        break

    elif key == 'b':
        print("Brisanje tocaka.")
        points = np.empty((0, 2), dtype=int)       
      
    elif key == 'q':
        print("Izlazak iz programa.")
        break
