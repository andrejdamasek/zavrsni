#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from core.real_ur5_controller import UR5Controller 
import threading


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
        if point is None:
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
    T_6_0[0, 3] = 0  # X
    T_6_0[1, 3] = 0.6   # Y
    T_6_0[2, 3] = 0.45 # Z

    T_6_0[:3, :3] = np.array([[1, 0, 0],
                              [0, -1, 0],
                              [0, 0, -1]])

    current_joints = controller.get_current_joint_values()
    traj = [current_joints]

    joint_sol = controller.get_closest_ik_solution(T_6_0)

    if joint_sol is not None:
        traj.append(joint_sol.tolist())        
        controller.send_joint_trajectory_action(np.array(traj), max_velocity=0.5, max_acceleration=0.5)
        rospy.sleep(2)
        print("Robot postavljen u početnu poziciju.")
    else:
        rospy.logwarn("Nije pronađeno IK rješenje za početnu poziciju.")


def monitor_force_and_cancel(robot: UR5Controller, threshold: float = 30.0, check_rate: float = 50.0):
    """
    Monitors force in the background while a trajectory is active.
    Cancels the trajectory if force exceeds the threshold.
    """
    rate = rospy.Rate(check_rate)
    max_wait_cycles = int(2.0 * check_rate)  # 2 seconds timeout

    rospy.loginfo("[monitor] Waiting for trajectory to become ACTIVE...")

    # Phase 1: Wait for the goal to be accepted (state becomes ACTIVE)
    for _ in range(max_wait_cycles):
        state = robot.client.get_state()
        if state == 1:  # ACTIVE
            rospy.loginfo("[monitor] Trajectory is ACTIVE.")
            break
        elif state == 0:  # PENDING
            rate.sleep()
        else:
            rospy.loginfo_throttle(0.5, "[monitor] Goal state: %d (waiting for ACTIVE...)", state)
            rate.sleep()
    else:
        rospy.logwarn("[monitor] Timeout waiting for trajectory to become ACTIVE. Final state: %s", str(state))
        return

    # Phase 2: Monitor force while ACTIVE
    while not rospy.is_shutdown() and robot.client.get_state() == 1:
        wrench = robot.get_current_wrench()
        force = np.linalg.norm(wrench[:3])
        rospy.loginfo_throttle(1.0, "[monitor] Force: %.2f N", force)

        if force > threshold:
            rospy.logwarn("Force threshold exceeded: %.2f N", force)
            robot.force_violation = True
            robot.cancel_trajectory()
            return

        rate.sleep()

    rospy.loginfo("[monitor] Trajectory no longer ACTIVE. Monitoring stopped.")



def get_points_going_down(points_3d):
    points_going_down = []

    for i in range(len(points_3d)):
        if i == 0:
            points_going_down.append(points_3d[i])
        elif points_3d[i - 1] is None:
            points_going_down.append(points_3d[i])
    
    return points_going_down


def initialize_traj(points_3d):
    counter = 1
    traj = []
    for point in points_3d:
        if point is None:
            traj.append([])
            counter += 1

    traj.append([])

    return traj , counter


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

            print("Robot ide u startnu poziciju.")
            send_robot_to_start_position(controller)
           
            points_3d=transform_to_3d(points)   

            segments = sample_between_none(points_3d, step=5)       

            points_3d_going_down=get_points_going_down(points_3d)

            traj , counter= initialize_traj(points_3d)            
            
           

            z_up = 0.05 
            z = 0 

            current_pose = controller.get_current_tool_pose()
            T_6_0 = np.copy(current_pose)  
            T_6_0[:3, :3] = np.array([[1, 0, 0],
                                     [0, -1, 0],
                                     [0, 0, -1]])
            
            current_joints = controller.get_current_joint_values() #neznam jer mi je ovo potrebno
            traj[0]=[current_joints] #neznam jer mi je ovo potrebno

            for i in range(counter):

                point=points_3d_going_down[i]

               # T_6_0[:3, :3] = np.array([[1, 0, 0],
               #                         [0, -1, 0],
               #                         [0, 0, -1]])

                T_6_0[0, 3] = point[0] #x
                T_6_0[1, 3] = point[1] #y
                
                joint_sol = controller.get_closest_ik_solution(T_6_0)
                if joint_sol is not None:
                
                    controller.force_violation = False  
                    controller.zero_ft_sensor()
                    monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(controller, 7.0)) #sila 7.0 N
                    monitor_thread.start()
                    success = controller.send_joint_trajectory_action(np.array([controller.get_current_joint_values(), joint_sol]), max_velocity=0.5, max_acceleration=0.5)                     
                    monitor_thread.join()
                        
                    if controller.force_violation:
                        rospy.logwarn("Prekinuto zbog prevelike sile pri spuštanju.")
                    
                if joint_sol is None:
                    rospy.logwarn("No IK solution found for lower position.")

                if not controller.force_violation:  
                    current_pose = controller.get_current_tool_pose()
                    T_6_0 = np.copy(current_pose)
                    z=T_6_0[2, 3] # z os nakon spuštanja

                    for p in segments[i]:
                        if p is None:
                            T_6_0[2, 3]= z + z_up 
                            joint_sol = controller.get_closest_ik_solution(T_6_0)
                            if joint_sol is not None:
                                traj[i].append(joint_sol.tolist())
                            else:
                                rospy.logwarn("No IK solution found for offset pose.")
                        else:
                            T_6_0[0, 3] = p[0]
                            T_6_0[1, 3] = p[1]
                            T_6_0[2, 3] = z
                            joint_sol = controller.get_closest_ik_solution(T_6_0)
                            if joint_sol is not None:
                                traj[i].append(joint_sol.tolist())
                            else:
                                rospy.logwarn("No IK solution found for lower position.")

                    controller.send_joint_trajectory_action(np.array(traj[i]), max_velocity=0.5, max_acceleration=0.5)

            send_robot_to_start_position(controller)  #na kraju se vrati u startnu poziciju
            rospy.loginfo("UR5 simple demo finished.") 

if __name__ == "__main__":
    main()
