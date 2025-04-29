#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from core.real_ur5_controller import UR5Controller  

points = np.empty((0, 2), dtype=int) 
is_drawing = False

def save_points_txt(filename="points.txt"):
    global points
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
        if points.shape[0] > 0: 
            points = np.vstack((points, [-1, -1]))
        points = np.vstack((points, [x, y]))

    elif event == cv2.EVENT_MOUSEMOVE and is_drawing:
        points = np.vstack((points, [x, y]))

    elif event == cv2.EVENT_LBUTTONUP:
        is_drawing = False

def sample_between_none(points, step=5):
    result = []
    segment = []
    
    for point in points:
        if np.array_equal(point, [-1, -1]):
            if segment:
                sampled = segment[::step]
                if not np.array_equal(segment[-1], sampled[-1]):
                    sampled.append(segment[-1])  
                result.extend(sampled)
                segment = []
            result.append(point)
        else:
            segment.append(point)
    
    if segment:
        sampled = segment[::step]
        if not np.array_equal(segment[-1], sampled[-1]):
            sampled.append(segment[-1])
        result.extend(sampled)

    return np.array(result)
    
def transform_to_3d(points, scale=0.001, z_height=0, offsets=(-0.25, 0.8, 0)):
    X_offset, Y_offset, Z_offset = offsets
    points_3d = []

    for point in points:
        if np.all(point == [-1, -1]): 
            points_3d.append(None)
        else:
            x_3d = point[0] * scale  + X_offset
            y_3d = -point[1] * scale + Y_offset
            z_3d = z_height + Z_offset
            points_3d.append((x_3d, y_3d, z_3d))

    return points_3d

def send_robot_to_start_position(controller):
    current_pose = controller.get_current_tool_pose()
    T_6_0 = np.copy(current_pose)
    T_6_0[0, 3] = -0.25   # X
    T_6_0[1, 3] = 0.8   # Y
    T_6_0[2, 3] = 0.3   # Z

    T_6_0[:3, :3] = np.array([[1, 0, 0],
                              [0, -1, 0],
                              [0, 0, -1]])

    start_joints = controller.get_closest_ik_solution(T_6_0)

    if start_joints is not None:
        traj = [controller.get_current_joint_values(), start_joints]
        controller.send_joint_trajectory_action(np.array(traj), max_velocity=0.5, max_acceleration=0.5)
        rospy.sleep(2)
        print("Robot postavljen u početnu poziciju.")
    else:
        rospy.logwarn("Nije pronađeno IK rješenje za početnu poziciju.")


def main():
    global is_drawing, points

    while True:
    
        print("Pritisni 'w' za početak crtanja, 'r' za pokretanje robota, 's' za spremanje, 'l' za učitavanje, 'b' za brisanje, 'q' za izlaz.")
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
        

        elif key == 'b':
            print("Brisanje tocaka.")
            points = np.empty((0, 2), dtype=int)       
      
        elif key == 'q':
            print("Izlazak iz programa.")
            break


        elif key == 'r':
            
            print("Pokretanje robota.")
            
            rospy.init_node("ur5_simple_demo")
            controller = UR5Controller()
            rospy.loginfo("Starting UR5 simple demo...")

            send_robot_to_start_position(controller)


            current_pose = controller.get_current_tool_pose()
            T_6_0 = np.copy(current_pose)

            z_up = 0.05 
            pen_is_down = False  
            T_6_0[:3, :3] = np.array([[1, 0, 0],
                                     [0, -1, 0],
                                     [0, 0, -1]])

           
            new_points = sample_between_none(points, step=5)
            points_3d=transform_to_3d(new_points)
            print("Pokretanje robota.")
            current_joints = controller.get_current_joint_values()
            traj = [current_joints]

           
            for point in points_3d:
                if point is None:
                    if pen_is_down:
                        T_6_0[2, 3] += z_up 
                        joint_sol = controller.get_closest_ik_solution(T_6_0)
                        if joint_sol is not None:
                            traj.append(joint_sol.tolist())
                        else:
                            rospy.logwarn("No IK solution found for offset pose.")
                        pen_is_down = False

                else:                    
                    T_6_0[0, 3] = point[0]
                    T_6_0[1, 3] = point[1]
                    if not pen_is_down:
                        T_6_0[2, 3] -= z_up 
                        pen_is_down = True
                    
                    joint_sol = controller.get_closest_ik_solution(T_6_0)
                    if joint_sol is not None:
                        traj.append(joint_sol.tolist())

                rospy.sleep(1)
            
            controller.send_joint_trajectory_action(np.array(traj), max_velocity=0.5, max_acceleration=0.5)
            controller.shutdown()                     
        

if __name__ == "__main__":
    main()