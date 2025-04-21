from PyQt5.QtCore import QCoreApplication, QMetaObject, QRect, Qt, QThread
from PyQt5.QtGui import QIcon, QPixmap , QFont ,QFontDatabase
from PyQt5.QtWidgets import QLabel, QPushButton , QSlider , QComboBox
from utils import create_ssh_client, send_command, reset_cameras, scale
from sensor_msgs.msg import Imu
import os
from utils import BG_path , ROV_path
from stylesheet import Copilot_st1, Copilot_st2, apply_st , red_button , back_st, selection_st
from utils import reconnect_command


CAM_PORTS = {
    "Main": ["/dev/video0", "rtsp://192.168.191.56:8554/camerafeed1"],
    "Tilt": ["/dev/video0", "rtsp://192.168.191.56:8554/camerafeed1"],
    "Side": ["/dev/video0", "rtsp://192.168.191.56:8554/camerafeed1"],
    "Gripper L": ["/dev/video0", "rtsp://192.168.191.56:8554/camerafeed1"],
    "Gripper R": ["/dev/video0", "rtsp://192.168.191.56:8554/camerafeed1"]
}

class RestreamThread(QThread):
    def __init__(self, client, cam_port):
        super().__init__()
        self.client = client
        self.cam_port = cam_port

    def run(self):
        reconnect_command(self.client, self.cam_port)


class CopilotUi(object):
        
    #unhash line 26 here when testing on rpi 
    def __init__(self, ip, username, password):
        self.ip = ip
        self.username = username
        self.password = password
        #self.client = create_ssh_client(ip, username, password)
    def setupUi(self, Dialog):
        #loading font
        script_dir = os.path.dirname(os.path.abspath(__file__))
        font_path = os.path.join(script_dir, "GillSans.ttf")
        id = QFontDatabase.addApplicationFont(font_path)
        if id == -1:
            print("Failed to load font!")
        
        families = QFontDatabase.applicationFontFamilies(id)
        font=QFont(families[0],13)
        Afont=QFont(families[0],11)
        Dialog.resize(scale(929), scale(597))
        # background label
        self.BG_label = QLabel(Dialog)
        self.BG_label.setObjectName("Background")
        self.BG_label.setGeometry(QRect(scale(-3), scale(-5), scale(945), scale(607)))
        self.BG_label.setPixmap(QPixmap(BG_path))
        self.BG_label.setScaledContents(True)



        # I made these (D)labels only for design purposes, no ROS interaction here
        self.Dlabel = QLabel(Dialog)
        self.Dlabel.setObjectName("Dlabel")
        self.Dlabel.setGeometry(QRect(scale(20), scale(80), scale(101), scale(41)))
        self.Dlabel.setStyleSheet(Copilot_st1)
        self.Dlabel.setFont(font)


        self.Dlabel_2 = QLabel(Dialog)
        self.Dlabel_2.setObjectName("Dlabel 2")
        self.Dlabel_2.setGeometry(QRect(scale(20), scale(150), scale(111), scale(41)))
        self.Dlabel_2.setStyleSheet(Copilot_st1)
        self.Dlabel_2.setFont(font)

        self.Dlabel_3 = QLabel(Dialog)
        self.Dlabel_3.setObjectName("Dlabel 3")
        self.Dlabel_3.setGeometry(QRect(scale(20), scale(220), scale(111), scale(41)))
        self.Dlabel_3.setStyleSheet(Copilot_st1)
        self.Dlabel_3.setFont(font)

        self.Dlabel_4 = QLabel(Dialog)
        self.Dlabel_4.setObjectName("Dlabel 4")
        self.Dlabel_4.setGeometry(QRect(scale(20), scale(290), scale(101), scale(41)))
        self.Dlabel_4.setStyleSheet(Copilot_st1)
        self.Dlabel_4.setFont(font)
        
        self.Dlabel_5 = QLabel(Dialog)
        self.Dlabel_5.setObjectName("Dlabel 5")
        self.Dlabel_5.setGeometry(QRect(scale(20), scale(360), scale(101), scale(41)))
        self.Dlabel_5.setStyleSheet(Copilot_st1)
        self.Dlabel_5.setFont(font)

        self.Dlabel_6 = QLabel(Dialog)
        self.Dlabel_6.setObjectName("Dlabel 6")
        self.Dlabel_6.setGeometry(QRect(scale(20), scale(430), scale(101), scale(41)))
        self.Dlabel_6.setStyleSheet(Copilot_st1)
        self.Dlabel_6.setFont(font)

        self.Dlabel_7 = QLabel(Dialog)
        self.Dlabel_7.setObjectName("Dlabel 7")
        self.Dlabel_7.setGeometry(QRect(scale(20), scale(500), scale(121), scale(41)))
        self.Dlabel_7.setStyleSheet(Copilot_st1)
        self.Dlabel_7.setFont(font)

        # main labels here
        self.vx_label = QLabel(Dialog)
        self.vx_label.setObjectName("Vx label")
        self.vx_label.setGeometry(QRect(scale(90), scale(80), scale(151), scale(41)))
        self.vx_label.setStyleSheet(Copilot_st2)
        self.vx_label.setFont(font)

        self.vy_label = QLabel(Dialog)
        self.vy_label.setObjectName("Vy label")
        self.vy_label.setGeometry(QRect(scale(90), scale(150), scale(151), scale(41)))
        self.vy_label.setStyleSheet(Copilot_st2)
        self.vy_label.setFont(font)

        self.wz_label = QLabel(Dialog)
        self.wz_label.setObjectName("Wz label")
        self.wz_label.setGeometry(QRect(scale(90), scale(220), scale(151), scale(41)))
        self.wz_label.setStyleSheet(Copilot_st2)
        self.wz_label.setFont(font)

        self.roll_label = QLabel(Dialog)
        self.roll_label.setObjectName("Roll label")
        self.roll_label.setGeometry(QRect(scale(90), scale(290), scale(151), scale(41)))
        self.roll_label.setStyleSheet(Copilot_st2)
        self.roll_label.setFont(font)

        self.pitch_label = QLabel(Dialog)
        self.pitch_label.setObjectName("Pitch Label")
        self.pitch_label.setGeometry(QRect(scale(90), scale(360), scale(151), scale(41)))
        self.pitch_label.setStyleSheet(Copilot_st2)
        self.pitch_label.setFont(font)

        self.yaw_label = QLabel(Dialog)
        self.yaw_label.setObjectName("Yaw label")
        self.yaw_label.setGeometry(QRect(scale(90), scale(430), scale(151), scale(41)))
        self.yaw_label.setStyleSheet(Copilot_st2)
        self.yaw_label.setFont(font)

        self.depth_label = QLabel(Dialog)
        self.depth_label.setObjectName("Depth label")
        self.depth_label.setGeometry(QRect(scale(90), scale(500), scale(151), scale(41)))
        self.depth_label.setStyleSheet(Copilot_st2)
        self.depth_label.setFont(font)

        # rov figure label
        self.rov_label = QLabel(Dialog)
        self.rov_label.setObjectName("Rov image label")
        self.rov_label.setGeometry(QRect(scale(400), scale(250), scale(421), scale(291)))
        self.rov_label.setPixmap(QPixmap(ROV_path))
        self.rov_label.setScaledContents(True)

        # thrusters labels 
        self.th1 = QLabel(Dialog)
        self.th1.setObjectName("thruster 1 label")
        self.th1.setGeometry(QRect(scale(420), scale(320), scale(81), scale(41)))
        self.th1.setStyleSheet(Copilot_st1)
        self.th1.setFont(font)

        self.th2 = QLabel(Dialog)
        self.th2.setObjectName("thruster 2 label")
        self.th2.setGeometry(QRect(scale(420), scale(410), scale(81), scale(41)))
        self.th2.setStyleSheet(Copilot_st1)
        self.th2.setFont(font)


        self.th3 = QLabel(Dialog)
        self.th3.setObjectName("thruster 3 label")
        self.th3.setGeometry(QRect(scale(500), scale(470), scale(81), scale(41)))
        self.th3.setStyleSheet(Copilot_st1)
        self.th3.setFont(font)

        self.th4 = QLabel(Dialog)
        self.th4.setObjectName("thruster 4 labe")
        self.th4.setGeometry(QRect(scale(530), scale(280), scale(81), scale(41)))
        self.th4.setStyleSheet(Copilot_st1)
        self.th4.setFont(font)

        self.th5 = QLabel(Dialog)
        self.th5.setObjectName("thruster 5 labe")
        self.th5.setGeometry(QRect(scale(650), scale(300), scale(81), scale(41)))
        self.th5.setStyleSheet(Copilot_st1)
        self.th5.setFont(font)

        self.th6 = QLabel(Dialog)
        self.th6.setObjectName("thruster 6 label")
        self.th6.setGeometry(QRect(scale(650), scale(470), scale(81), scale(41)))
        self.th6.setStyleSheet(Copilot_st1)
        self.th6.setFont(font)

        self.th7 = QLabel(Dialog)
        self.th7.setObjectName("thruster 7 label")
        self.th7.setGeometry(QRect(scale(710), scale(380), scale(81), scale(41)))
        self.th7.setStyleSheet(Copilot_st1)
        self.th7.setFont(font)

        self.back_button = QPushButton(Dialog)
        self.back_button.setObjectName("pushButton")
        self.back_button.setGeometry(QRect(scale(10), scale(10), scale(61), scale(41)))
        icon = QIcon.fromTheme("go-previous")
        self.back_button.setIcon(icon)
        self.back_button.setStyleSheet(back_st)
        self.back_button.setFont(QFont("Gill Sans",12))

        #Camera adjusting system labels & buttons

        #Another Dlabel for design purposes 
        self.Dlabel_8 = QLabel(Dialog)
        self.Dlabel_8.setObjectName("(Design) label")
        self.Dlabel_8.setGeometry(QRect(scale(260), scale(80), scale(651), scale(171)))
        self.Dlabel_8.setStyleSheet(Copilot_st2)


        self.StreamButton = QPushButton(Dialog)
        self.StreamButton.setObjectName("Stop Recording for Photosphere Task")
        self.StreamButton.setGeometry(QRect(scale(800),scale(149) ,scale(101), scale(41)))
        self.StreamButton.setStyleSheet(red_button)
        self.StreamButton.setFont(Afont)
        self.StreamButton.clicked.connect(self.Stream_cam_clicked)

        self.CAS = QLabel(Dialog)
        self.CAS.setObjectName("Camera Adjusting system label")
        self.CAS.setGeometry(QRect(scale(500), scale(60), scale(221), scale(41)))
        self.CAS.setStyleSheet(Copilot_st1)
        self.CAS.setFont(font)

        #brightness adjusting 
        self.brightness = QLabel(Dialog)
        self.brightness.setObjectName("brightness label")
        self.brightness.setGeometry(QRect(scale(300), scale(110), scale(121), scale(41)))
        self.brightness.setStyleSheet(Copilot_st1)
        self.brightness.setFont(font)

        self.brightness_slider = QSlider(Dialog)
        self.brightness_slider.setObjectName("slider for changing the brightness")
        self.brightness_slider.setGeometry(QRect(scale(280), scale(160), scale(160), scale(25)))
        self.brightness_slider.setOrientation(Qt.Orientation.Horizontal)

        #made the slider start at the middle to increase or decrease the brightness instead of only increasing 

        self.brightness_slider.setMinimum(0) 
        self.brightness_slider.setMaximum(255)   
        self.brightness_slider.setValue(128)

        self.apply_brightness = QPushButton(Dialog)
        self.apply_brightness.setObjectName("apply button for brightness")
        self.apply_brightness.setGeometry(QRect(scale(310), scale(200), scale(100), scale(32)))
        self.apply_brightness.setStyleSheet(apply_st)
        self.apply_brightness.setFont(Afont)
        self.apply_brightness.clicked.connect(self.apply_brightness_clicked)

        #Contrast adjusting
        self.contrast = QLabel(Dialog)
        self.contrast.setObjectName("Contrast label")
        self.contrast.setGeometry(QRect(scale(470), scale(110), scale(121), scale(41)))
        self.contrast.setStyleSheet(Copilot_st1)
        self.contrast.setFont(font)

        self.contrast_slider = QSlider(Dialog)
        self.contrast_slider.setObjectName("slider for changing the contrast")
        self.contrast_slider.setGeometry(QRect(scale(450), scale(160), scale(160), scale(25)))
        self.contrast_slider.setOrientation(Qt.Orientation.Horizontal)

        self.contrast_slider.setMinimum(0)
        self.contrast_slider.setMaximum(255)
        self.contrast_slider.setValue(32)
        
        self.apply_contrast = QPushButton(Dialog)
        self.apply_contrast.setObjectName("apply button for contrast")
        self.apply_contrast.setGeometry(QRect(scale(480), scale(200), scale(100), scale(32)))
        self.apply_contrast.setStyleSheet(apply_st)
        self.apply_contrast.setFont(Afont)
        self.apply_contrast.clicked.connect(self.apply_contrast_clicked)

        #Backlight adjusting
        self.backLight = QLabel(Dialog)
        self.backLight.setObjectName("Back light label")
        self.backLight.setGeometry(QRect(scale(640), scale(110), scale(121), scale(41)))
        self.backLight.setStyleSheet(Copilot_st1)
        self.backLight.setFont(font)

        self.backLight_slider = QSlider(Dialog)
        self.backLight_slider.setObjectName("slider for changing the contrast")
        self.backLight_slider.setGeometry(QRect(scale(620), scale(160), scale(160), scale(25)))
        self.backLight_slider.setOrientation(Qt.Orientation.Horizontal)

        self.backLight_slider.setMinimum(0)
        self.backLight_slider.setMaximum(2)
        self.backLight_slider.setValue(0)

        self.apply_backlight = QPushButton(Dialog)
        self.apply_backlight.setObjectName("apply button for back light")
        self.apply_backlight.setGeometry(QRect(scale(660), scale(200), scale(100), scale(32)))
        self.apply_backlight.setStyleSheet(apply_st)
        self.apply_backlight.setFont(Afont)
        self.apply_backlight.clicked.connect(self.apply_backlight_clicked)

        #Reset button
        self.reset = QPushButton(Dialog)
        self.reset.setObjectName("reset button")
        self.reset.setGeometry(QRect(scale(800), scale(100), scale(101), scale(41)))
        self.reset.setStyleSheet(red_button)
        self.reset.setFont(font)
        self.reset.clicked.connect(self.reset_clicked)


        #Select Box
        self.comboBox = QComboBox(Dialog)
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.setObjectName("selection box")
        self.comboBox.setGeometry(QRect(scale(800), scale(200), scale(103), scale(32)))
        self.comboBox.setStyleSheet(selection_st)
        self.comboBox.setFont(Afont)
        self.setText(Dialog)
        QMetaObject.connectSlotsByName(Dialog)

    def apply_brightness_clicked(self):
        cam_name = self.comboBox.currentText()
        if cam_name != "Select":
            value = self.brightness_slider.value()
            send_command(self.client, CAM_PORTS[cam_name], "brightness", value)

    def apply_contrast_clicked(self):
        cam_name = self.comboBox.currentText()
        if cam_name != "Select":
            value = self.contrast_slider.value()
            send_command(self.client, CAM_PORTS[cam_name], "contrast", value)

    def apply_backlight_clicked(self):
        cam_name = self.comboBox.currentText()
        if cam_name != "Select":
            value = self.backLight_slider.value()
            send_command(self.client, CAM_PORTS[cam_name], "backlight_compensation", value)

    def reset_clicked(self):
        cam_name = self.comboBox.currentText()
        if cam_name != "Select":
            reset_cameras(self.client, CAM_PORTS[cam_name])
        self.brightness_slider.setValue(128)
        self.contrast_slider.setValue(32)
        self.backLight_slider.setValue(0)
    
    def Stream_cam_clicked(self):
        cam_name = self.comboBox.currentText()
        if cam_name != "Select":
            self.restream_thread = RestreamThread(self.client, CAM_PORTS[cam_name])
            self.restream_thread.start()

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.BG_label.setText("")
        self.Dlabel.setText(QCoreApplication.translate("Dialog", "  Vx", None))
        self.Dlabel_2.setText(QCoreApplication.translate("Dialog", "  Vy", None))
        self.Dlabel_3.setText(QCoreApplication.translate("Dialog", " Wz", None))
        self.Dlabel_4.setText(QCoreApplication.translate("Dialog", "Roll", None))
        self.Dlabel_5.setText(QCoreApplication.translate("Dialog", "Pitch", None))
        self.Dlabel_6.setText(QCoreApplication.translate("Dialog", "Yaw ", None))
        self.Dlabel_7.setText(QCoreApplication.translate("Dialog", "Depth ", None))


        self.vx_label.setText(QCoreApplication.translate("Dialog", "...", None))
        self.vy_label.setText(QCoreApplication.translate("Dialog", "...", None))
        self.wz_label.setText(QCoreApplication.translate("Dialog", "...", None))
        self.roll_label.setText(QCoreApplication.translate("Dialog", "...", None))
        self.pitch_label.setText(QCoreApplication.translate("Dialog", "...", None))
        self.yaw_label.setText(QCoreApplication.translate("Dialog", "...", None))
        self.depth_label.setText(QCoreApplication.translate("Dialog", "...", None))
        self.StreamButton.setText(QCoreApplication.translate("Dialog", "Stream", None))
        self.rov_label.setText("")


        self.th1.setText(QCoreApplication.translate("Dialog", "Th1", None))
        self.th2.setText(QCoreApplication.translate("Dialog", "Th2", None))
        self.th3.setText(QCoreApplication.translate("Dialog", "Th3", None))
        self.th4.setText(QCoreApplication.translate("Dialog", "Th6", None))
        self.th5.setText(QCoreApplication.translate("Dialog", "Th5", None))
        self.th6.setText(QCoreApplication.translate("Dialog", "Th4", None))
        self.th7.setText(QCoreApplication.translate("Dialog", "Th7", None))


        self.Dlabel_8.setText(QCoreApplication.translate("Dialog", "", None))
        self.CAS.setText(QCoreApplication.translate("Dialog", "Camera Adjusting System", None))
        self.brightness.setText(QCoreApplication.translate("Dialog", "Brightness", None))
        self.contrast.setText(QCoreApplication.translate("Dialog", " Contrast", None))
        self.backLight.setText(QCoreApplication.translate("Dialog", "BackLight", None))
        self.apply_brightness.setText(QCoreApplication.translate("Dialog", "Apply", None))
        self.apply_contrast.setText(QCoreApplication.translate("Dialog", "Apply", None))
        self.apply_backlight.setText(QCoreApplication.translate("Dialog", "Apply", None))
        self.reset.setText(QCoreApplication.translate("Dialog", "Reset", None))
        self.comboBox.setItemText(0, QCoreApplication.translate("Dialog", "Select", None))
        self.comboBox.setItemText(1, QCoreApplication.translate("Dialog", "Main", None))
        self.comboBox.setItemText(2, QCoreApplication.translate("Dialog", "Tilt", None))
        self.comboBox.setItemText(3, QCoreApplication.translate("Dialog", "Side", None))
        self.comboBox.setItemText(4, QCoreApplication.translate("Dialog", "Gripper L", None))
        self.comboBox.setItemText(5, QCoreApplication.translate("Dialog", "Gripper R", None))
        self.comboBox.setItemText(6, QCoreApplication.translate("Dialog", "Bottom", None))
        
        self.back_button.setText(QCoreApplication.translate("Dialog","Back", None))

    # Standalone function to update all ROV labels
    def update_imu(self,
        imu_msg):
        """
        Update IMU labels for linear acceleration and orientation.
        """
        # Linear acceleration
        self.vx_label.setText(f"{imu_msg.linear_acceleration.x:.2f}")
        self.vy_label.setText(f"{imu_msg.linear_acceleration.y:.2f}")
        self.wz_label.setText(f"{imu_msg.linear_acceleration.z:.2f}")
        # Orientation (quaternion components)
        self.roll_label.setText(f"{imu_msg.orientation.x:.2f}")
        self.pitch_label.setText(f"{imu_msg.orientation.y:.2f}")
        self.yaw_label.setText(f"{imu_msg.orientation.z:.2f}")



    def update_depth(self,
        depth
    ):
        """
        Update the depth label.
        """
        self.depth_label.setText(f"{depth:.2f}")

    def update_gripper_r(
            self,
            gripper_r
        ):
            """
            Update the gripper right label.
            """
            self.th1.setText(f"{gripper_r}")

    def update_gripper_l(
            self,
            gripper_l
        ):
            """
            Update the gripper left label.
            """
            self.th2.setText(f"{gripper_l}")

    def update_thrusters(
            self,
            thruster_values
        ):
            """
            Update each thruster current label.
            """
            self.th1.setText(f"{thruster_values[0]}")
            self.th2.setText(f"{thruster_values[1]}")
            self.th3.setText(f"{thruster_values[2]}")
            self.th4.setText(f"{thruster_values[3]}")
            self.th5.setText(f"{thruster_values[4]}")
            self.th6.setText(f"{thruster_values[5]}")
            self.th7.setText(f"{thruster_values[6]}")