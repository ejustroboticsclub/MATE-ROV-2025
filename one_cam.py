
import subprocess


while True:
    cam = input("Enter camera number (0-4) \n 0: Top left (Net), 1: Top right (Side), 2: Bottom left (Gripper), 3: Bottom right (Jelly), 4: Middle ZED\n")
    if cam == 'q':
        break

    cam = int(cam)
    if cam < 0 or cam > 4:
        print("Invalid camera number. Please enter a number between 0 and 4.")
        continue
    # Run the file run_one_cam.py with the selected camera number
    subprocess.run(["python3", "run_one_cam.py", "--cam_index", str(cam)])
    
    
    
