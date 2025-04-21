#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from core.real_ur5_controller import UR5Controller  # Assuming your class is in this file

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
        points = np.vstack((points, [-1, -1]))
        points = np.vstack((points, [x, y]))
    elif event == cv2.EVENT_MOUSEMOVE and is_drawing:
        points = np.vstack((points, [x, y]))
    elif event == cv2.EVENT_LBUTTONUP:
        is_drawing = False



    
def transform_to_3d(scale=0.1, z_height=0, offsets=(0, 0.5, 0)):
    X_offset, Y_offset, Z_offset = offsets
    global points
    points_3d = []

    for point in points:
        if np.all(point == [-1, -1]): 
            points_3d.append(None)
        else:
            x_3d = point[0] * scale  + X_offset
            y_3d = point[1] * scale+ Y_offset
            z_3d = z_height + Z_offset
            points_3d.append((x_3d, y_3d, z_3d))
    return points_3d

def main():
    global is_drawing 
    global points
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

            
            current_pose = controller.get_current_tool_pose()
            T_6_0 = np.copy(current_pose)

            T_6_0[0,3]+=0.1 # move +10 cm in X direction
            T_6_0[2, 3] += 0.10  # move +10 cm in Z direction

            rospy.loginfo("Planning Cartesian move 10 cm in X and Z...")
            joint_sol = controller.get_closest_ik_solution(T_6_0)
            #joint_sol[0] += np.pi
            #joint_sol[joint_sol>np.pi]-=(2.0*np.pi)
            #joint_sol[joint_sol<-np.pi]+=(2.0*np.pi)
            if joint_sol is not None:
               current_joints = controller.get_current_joint_values()
               joint_traj = np.array([current_joints, joint_sol])
               controller.send_joint_trajectory_action(joint_traj, max_velocity=0.5, max_acceleration=0.5)
            else:
               rospy.logwarn("No IK solution found for offset pose.")

            rospy.sleep(1.0)

            T_6_0[1,3]+=0.1 # move +10 cm in y direction
            T_6_0[2, 3] -= 0.10  # move +10 cm in Z direction
            rospy.loginfo("Planning Cartesian move 10 cm in y and -10cm in Z...")
            joint_sol = controller.get_closest_ik_solution(T_6_0)
            #joint_sol[0] += np.pi
            #joint_sol[joint_sol>np.pi]-=(2.0*np.pi)
            #joint_sol[joint_sol<-np.pi]+=(2.0*np.pi)
            if joint_sol is not None:
               current_joints = controller.get_current_joint_values()
               joint_traj = np.array([current_joints, joint_sol])
               controller.send_joint_trajectory_action(joint_traj, max_velocity=0.5, max_acceleration=0.5)
            else:
               rospy.logwarn("No IK solution found for offset pose.")

            rospy.sleep(1.0)
            controller.shutdown()
            '''
            points_3d=transform_to_3d()
            for point in points_3d:
                if point is not None:
                    T_6_0[:3, 3] = point
                    rospy.loginfo("Planning Cartesian move...")
                    joint_sol = controller.get_closest_ik_solution(T_6_0)
                    if joint_sol is not None:
                        current_joints = controller.get_current_joint_values()
                        joint_traj = np.array([current_joints, joint_sol])
                        controller.send_joint_trajectory_action(joint_traj, max_velocity=0.5, max_acceleration=0.5)
                    else:
                        rospy.logwarn("No IK solution found for offset pose.")
                    rospy.sleep(1.0)
                    controller.shutdown()
                else:
                    T_6_0[2,3]+=0.05
                    rospy.loginfo("Planning Cartesian move...")
                    joint_sol = controller.get_closest_ik_solution(T_6_0)
                    if joint_sol is not None:
                        current_joints = controller.get_current_joint_values()
                        joint_traj = np.array([current_joints, joint_sol])
                        controller.send_joint_trajectory_action(joint_traj, max_velocity=0.5, max_acceleration=0.5)
            
            '''
            
            

if __name__ == "__main__":
    main()
