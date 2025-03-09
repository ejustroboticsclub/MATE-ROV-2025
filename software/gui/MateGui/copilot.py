from PyQt5.QtCore import QCoreApplication, QMetaObject, QRect, Qt
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QLabel, QPushButton

from stylesheet import Copilot_st1, Copilot_st2

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
        self.Dlabel.setGeometry(QRect(20, 100, 101, 41))
        self.Dlabel.setStyleSheet(Copilot_st1)
        self.Dlabel.setTextFormat(Qt.TextFormat.AutoText)

        self.Dlabel_2 = QLabel(Dialog)
        self.Dlabel_2.setObjectName("Dlabel 2")
        self.Dlabel_2.setGeometry(QRect(20, 150, 111, 41))
        self.Dlabel_2.setStyleSheet(Copilot_st1)

        self.Dlabel_3 = QLabel(Dialog)
        self.Dlabel_3.setObjectName("Dlabel 3")
        self.Dlabel_3.setGeometry(QRect(20, 200, 111, 41))
        self.Dlabel_3.setStyleSheet(Copilot_st1)

        self.Dlabel_4 = QLabel(Dialog)
        self.Dlabel_4.setObjectName("Dlabel 4")
        self.Dlabel_4.setGeometry(QRect(20, 250, 101, 41))
        self.Dlabel_4.setStyleSheet(Copilot_st1)
        
        self.Dlabel_5 = QLabel(Dialog)
        self.Dlabel_5.setObjectName("Dlabel 5")
        self.Dlabel_5.setGeometry(QRect(20, 300, 101, 41))
        self.Dlabel_5.setStyleSheet(Copilot_st1)

        self.Dlabel_6 = QLabel(Dialog)
        self.Dlabel_6.setObjectName("Dlabel 6")
        self.Dlabel_6.setGeometry(QRect(20, 350, 101, 41))
        self.Dlabel_6.setStyleSheet(Copilot_st1)

        self.Dlabel_7 = QLabel(Dialog)
        self.Dlabel_7.setObjectName("Dlabel 7")
        self.Dlabel_7.setGeometry(QRect(20, 400, 121, 41))
        self.Dlabel_7.setStyleSheet(Copilot_st1)

        # main labels here
        self.vx_label = QLabel(Dialog)
        self.vx_label.setObjectName("Vx label")
        self.vx_label.setGeometry(QRect(90, 100, 151, 41))
        self.vx_label.setStyleSheet(Copilot_st2)

        self.vy_label = QLabel(Dialog)
        self.vy_label.setObjectName("Vy label")
        self.vy_label.setGeometry(QRect(90, 150, 151, 41))
        self.vy_label.setStyleSheet(Copilot_st2)

        self.wz_label = QLabel(Dialog)
        self.wz_label.setObjectName("Wz label")
        self.wz_label.setGeometry(QRect(90, 200, 151, 41))
        self.wz_label.setStyleSheet(Copilot_st2)

        self.roll_label = QLabel(Dialog)
        self.roll_label.setObjectName("Roll label")
        self.roll_label.setGeometry(QRect(90, 250, 151, 41))
        self.roll_label.setStyleSheet(Copilot_st2)

        self.pitch_label = QLabel(Dialog)
        self.pitch_label.setObjectName("Pitch Label")
        self.pitch_label.setGeometry(QRect(90, 300, 151, 41))
        self.pitch_label.setStyleSheet(Copilot_st2)

        self.yaw_label = QLabel(Dialog)
        self.yaw_label.setObjectName("Yaw label")
        self.yaw_label.setGeometry(QRect(90, 350, 151, 41))
        self.yaw_label.setStyleSheet(Copilot_st2)

        self.depth_label = QLabel(Dialog)
        self.depth_label.setObjectName("Depth label")
        self.depth_label.setGeometry(QRect(90, 400, 151, 41))
        self.depth_label.setStyleSheet(Copilot_st2)

        # rov figure label
        self.rov_label = QLabel(Dialog)
        self.rov_label.setObjectName("Rov image label")
        self.rov_label.setGeometry(QRect(440, 20, 421, 291))
        self.rov_label.setPixmap(QPixmap(u"Visuals/rov(final).png"))
        self.rov_label.setScaledContents(True)

        # thrusters labels 
        self.th1 = QLabel(Dialog)
        self.th1.setObjectName("thruster 1 label")
        self.th1.setGeometry(QRect(460, 80, 81, 41))
        self.th1.setStyleSheet(Copilot_st1)

        self.th2 = QLabel(Dialog)
        self.th2.setObjectName("thruster 2 label")
        self.th2.setGeometry(QRect(460, 220, 81, 41))
        self.th2.setStyleSheet(Copilot_st1)

        self.th3 = QLabel(Dialog)
        self.th3.setObjectName("thruster 3 label")
        self.th3.setGeometry(QRect(580, 260, 81, 41))
        self.th3.setStyleSheet(Copilot_st1)

        self.th4 = QLabel(Dialog)
        self.th4.setObjectName("thruster 4 labe")
        self.th4.setGeometry(QRect(580, 40, 81, 41))
        self.th4.setStyleSheet(Copilot_st1)

        self.th5 = QLabel(Dialog)
        self.th5.setObjectName("thruster 5 labe")
        self.th5.setGeometry(QRect(740, 60, 81, 41))
        self.th5.setStyleSheet(Copilot_st1)

        self.th6 = QLabel(Dialog)
        self.th6.setObjectName("thruster 6 label")
        self.th6.setGeometry(QRect(740, 240, 81, 41))
        self.th6.setStyleSheet(Copilot_st1)

        self.back_button = QPushButton(Dialog)
        self.back_button.setObjectName("pushButton")
        self.back_button.setGeometry(QRect(10, 10, 61, 41))
        icon = QIcon.fromTheme("go-previous")
        self.back_button.setIcon(icon)

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
        
        self.back_button.setText(QCoreApplication.translate("Dialog","Back", None))



   