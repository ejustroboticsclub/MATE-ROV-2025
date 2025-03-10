from PyQt5.QtCore import QCoreApplication, QMetaObject, QRect, Qt
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QLabel, QPushButton , QSlider , QComboBox

from stylesheet import Copilot_st1, Copilot_st2, apply_st , red_button

class CopilotUi(object):
    def setupUi(self, Dialog):
        Dialog.resize(929, 597)
        
        # background label
        self.BG_label = QLabel(Dialog)
        self.BG_label.setObjectName("Background")
        self.BG_label.setGeometry(QRect(-3, -5, 931, 601))
        self.BG_label.setPixmap(QPixmap("Visuals/Background(final).jpg"))
        self.BG_label.setScaledContents(True)

        # I made these (D)labels only for design purposes, no ROS interaction here
        self.Dlabel = QLabel(Dialog)
        self.Dlabel.setObjectName("Dlabel")
        self.Dlabel.setGeometry(QRect(20, 80, 101, 41))
        self.Dlabel.setStyleSheet(Copilot_st1)
        self.Dlabel.setTextFormat(Qt.TextFormat.AutoText)

        self.Dlabel_2 = QLabel(Dialog)
        self.Dlabel_2.setObjectName("Dlabel 2")
        self.Dlabel_2.setGeometry(QRect(20, 150, 111, 41))
        self.Dlabel_2.setStyleSheet(Copilot_st1)

        self.Dlabel_3 = QLabel(Dialog)
        self.Dlabel_3.setObjectName("Dlabel 3")
        self.Dlabel_3.setGeometry(QRect(20, 220, 111, 41))
        self.Dlabel_3.setStyleSheet(Copilot_st1)

        self.Dlabel_4 = QLabel(Dialog)
        self.Dlabel_4.setObjectName("Dlabel 4")
        self.Dlabel_4.setGeometry(QRect(20, 290, 101, 41))
        self.Dlabel_4.setStyleSheet(Copilot_st1)
        
        self.Dlabel_5 = QLabel(Dialog)
        self.Dlabel_5.setObjectName("Dlabel 5")
        self.Dlabel_5.setGeometry(QRect(20, 360, 101, 41))
        self.Dlabel_5.setStyleSheet(Copilot_st1)

        self.Dlabel_6 = QLabel(Dialog)
        self.Dlabel_6.setObjectName("Dlabel 6")
        self.Dlabel_6.setGeometry(QRect(20, 430, 101, 41))
        self.Dlabel_6.setStyleSheet(Copilot_st1)

        self.Dlabel_7 = QLabel(Dialog)
        self.Dlabel_7.setObjectName("Dlabel 7")
        self.Dlabel_7.setGeometry(QRect(20, 500, 121, 41))
        self.Dlabel_7.setStyleSheet(Copilot_st1)

        # main labels here
        self.vx_label = QLabel(Dialog)
        self.vx_label.setObjectName("Vx label")
        self.vx_label.setGeometry(QRect(90, 80, 151, 41))
        self.vx_label.setStyleSheet(Copilot_st2)

        self.vy_label = QLabel(Dialog)
        self.vy_label.setObjectName("Vy label")
        self.vy_label.setGeometry(QRect(90, 150, 151, 41))
        self.vy_label.setStyleSheet(Copilot_st2)

        self.wz_label = QLabel(Dialog)
        self.wz_label.setObjectName("Wz label")
        self.wz_label.setGeometry(QRect(90, 220, 151, 41))
        self.wz_label.setStyleSheet(Copilot_st2)

        self.roll_label = QLabel(Dialog)
        self.roll_label.setObjectName("Roll label")
        self.roll_label.setGeometry(QRect(90, 290, 151, 41))
        self.roll_label.setStyleSheet(Copilot_st2)

        self.pitch_label = QLabel(Dialog)
        self.pitch_label.setObjectName("Pitch Label")
        self.pitch_label.setGeometry(QRect(90, 360, 151, 41))
        self.pitch_label.setStyleSheet(Copilot_st2)

        self.yaw_label = QLabel(Dialog)
        self.yaw_label.setObjectName("Yaw label")
        self.yaw_label.setGeometry(QRect(90, 430, 151, 41))
        self.yaw_label.setStyleSheet(Copilot_st2)

        self.depth_label = QLabel(Dialog)
        self.depth_label.setObjectName("Depth label")
        self.depth_label.setGeometry(QRect(90, 500, 151, 41))
        self.depth_label.setStyleSheet(Copilot_st2)

        # rov figure label
        self.rov_label = QLabel(Dialog)
        self.rov_label.setObjectName("Rov image label")
        self.rov_label.setGeometry(QRect(400, 250, 421, 291))
        self.rov_label.setPixmap(QPixmap(u"Visuals/ROV(final).png"))
        self.rov_label.setScaledContents(True)

        # thrusters labels 
        self.th1 = QLabel(Dialog)
        self.th1.setObjectName("thruster 1 label")
        self.th1.setGeometry(QRect(420, 310, 81, 41))
        self.th1.setStyleSheet(Copilot_st1)

        self.th2 = QLabel(Dialog)
        self.th2.setObjectName("thruster 2 label")
        self.th2.setGeometry(QRect(420, 450, 81, 41))
        self.th2.setStyleSheet(Copilot_st1)

        self.th3 = QLabel(Dialog)
        self.th3.setObjectName("thruster 3 label")
        self.th3.setGeometry(QRect(550, 490, 81, 41))
        self.th3.setStyleSheet(Copilot_st1)

        self.th4 = QLabel(Dialog)
        self.th4.setObjectName("thruster 4 labe")
        self.th4.setGeometry(QRect(690, 460, 81, 41))
        self.th4.setStyleSheet(Copilot_st1)

        self.th5 = QLabel(Dialog)
        self.th5.setObjectName("thruster 5 labe")
        self.th5.setGeometry(QRect(680, 300, 81, 41))
        self.th5.setStyleSheet(Copilot_st1)

        self.th6 = QLabel(Dialog)
        self.th6.setObjectName("thruster 6 label")
        self.th6.setGeometry(QRect(550, 270, 81, 41))
        self.th6.setStyleSheet(Copilot_st1)

        self.back_button = QPushButton(Dialog)
        self.back_button.setObjectName("pushButton")
        self.back_button.setGeometry(QRect(10, 10, 61, 41))
        icon = QIcon.fromTheme("go-previous")
        self.back_button.setIcon(icon)

        #Camera adjusting system labels & buttons

        #Another Dlabel for design purposes 
        self.Dlabel_8 = QLabel(Dialog)
        self.Dlabel_8.setObjectName("(Design) label")
        self.Dlabel_8.setGeometry(QRect(260, 80, 651, 171))
        self.Dlabel_8.setStyleSheet(Copilot_st2)

        self.CAS = QLabel(Dialog)
        self.CAS.setObjectName("Camera Adjusting system label")
        self.CAS.setGeometry(QRect(500, 60, 221, 41))
        self.CAS.setStyleSheet(Copilot_st1)

        #brightness adjusting 
        self.brightness = QLabel(Dialog)
        self.brightness.setObjectName("brightness label")
        self.brightness.setGeometry(QRect(300, 110, 121, 41))
        self.brightness.setStyleSheet(Copilot_st1)

        self.brightness_slider = QSlider(Dialog)
        self.brightness_slider.setObjectName("slider for changing the brightness")
        self.brightness_slider.setGeometry(QRect(280, 160, 160, 25))
        self.brightness_slider.setOrientation(Qt.Orientation.Horizontal)

        #made the slider start at the middle to increase or decrease the brightness instead of only increasing 

        self.brightness_slider.setMinimum(-100) 
        self.brightness_slider.setMaximum(100)   
        self.brightness_slider.setValue(0)

        self.apply_brightness = QPushButton(Dialog)
        self.apply_brightness.setObjectName("apply button for brightness")
        self.apply_brightness.setGeometry(QRect(310, 200, 100, 32))
        self.apply_brightness.setStyleSheet(apply_st)

        #Contrast adjusting
        self.contrast = QLabel(Dialog)
        self.contrast.setObjectName("Contrast label")
        self.contrast.setGeometry(QRect(470, 110, 121, 41))
        self.contrast.setStyleSheet(Copilot_st1)

        self.contrast_slider = QSlider(Dialog)
        self.contrast_slider.setObjectName("slider for changing the contrast")
        self.contrast_slider.setGeometry(QRect(450, 160, 160, 25))
        self.contrast_slider.setOrientation(Qt.Orientation.Horizontal)

        self.contrast_slider.setMinimum(-100)
        self.contrast_slider.setMaximum(100)
        self.contrast_slider.setValue(0)
        
        self.apply_contrast = QPushButton(Dialog)
        self.apply_contrast.setObjectName("apply button for contrast")
        self.apply_contrast.setGeometry(QRect(480, 200, 100, 32))
        self.apply_contrast.setStyleSheet(apply_st)

        #Backlight adjusting
        self.backLight = QLabel(Dialog)
        self.backLight.setObjectName("Back light label")
        self.backLight.setGeometry(QRect(640, 110, 121, 41))
        self.backLight.setStyleSheet(Copilot_st1)

        self.backLight_slider = QSlider(Dialog)
        self.backLight_slider.setObjectName("slider for changing the contrast")
        self.backLight_slider.setGeometry(QRect(620, 160, 160, 25))
        self.backLight_slider.setOrientation(Qt.Orientation.Horizontal)

        self.backLight_slider.setMinimum(-100)
        self.backLight_slider.setMaximum(100)
        self.backLight_slider.setValue(0)

        self.apply_backlight = QPushButton(Dialog)
        self.apply_backlight.setObjectName("apply button for back light")
        self.apply_backlight.setGeometry(QRect(660, 200, 100, 32))
        self.apply_backlight.setStyleSheet(apply_st)

        #Reset button
        self.reset = QPushButton(Dialog)
        self.reset.setObjectName("reset button")
        self.reset.setGeometry(QRect(810, 110, 91, 41))
        self.reset.setStyleSheet(red_button)

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
        self.comboBox.setGeometry(QRect(800, 200, 103, 32))
        
        self.setText(Dialog)
        QMetaObject.connectSlotsByName(Dialog)

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
        self.rov_label.setText("")


        self.th1.setText(QCoreApplication.translate("Dialog", "Th1", None))
        self.th2.setText(QCoreApplication.translate("Dialog", "Th2", None))
        self.th3.setText(QCoreApplication.translate("Dialog", "Th3", None))
        self.th4.setText(QCoreApplication.translate("Dialog", "Th6", None))
        self.th5.setText(QCoreApplication.translate("Dialog", "Th5", None))
        self.th6.setText(QCoreApplication.translate("Dialog", "Th4", None))

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



   