from PyQt5.QtCore import QCoreApplication, QMetaObject, QRect, Qt
from PyQt5.QtGui import QIcon, QPixmap, QFont, QFontDatabase
from PyQt5.QtWidgets import QLabel, QPushButton, QSlider, QComboBox, QFrame
from stylesheet import Copilot_st1, Copilot_st2, float_st , back_st , Engineer_buttons_st
import os
from utils import BG_path
class FloatUi(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(930, 599)
        #loading font
        script_dir = os.path.dirname(os.path.abspath(__file__))
        font_path = os.path.join(script_dir, "GillSans.ttf")
        id = QFontDatabase.addApplicationFont(font_path)
        if id == -1:
            print("Failed to load font!")
        
        families = QFontDatabase.applicationFontFamilies(id)
        font=QFont(families[0],13)
        Afont=QFont(families[0],15)
        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("BG label")
        self.Bg_label.setGeometry(QRect(-3, -5, 945, 607))
        self.Bg_label.setPixmap(QPixmap(BG_path))
        self.Bg_label.setScaledContents(True)

        self.Dlabel = QLabel(Dialog)
        self.Dlabel.setObjectName("Design label")
        self.Dlabel.setGeometry(QRect(190, 30, 551, 501))
        self.Dlabel.setStyleSheet(Copilot_st2)

        self.MainLine = QFrame(Dialog)
        self.MainLine.setObjectName(u"line_3")
        self.MainLine.setGeometry(QRect(380, 60, 151, 461))
        self.MainLine.setFrameShape(QFrame.Shape.VLine)
        self.MainLine.setFrameShadow(QFrame.Shadow.Sunken)

        self.DT = QLabel(Dialog)
        self.DT.setObjectName(u"label_3")
        self.DT.setGeometry(QRect(350, 10, 201, 41))
        self.DT.setStyleSheet(Copilot_st1)
        self.DT.setFont(Afont)

        self.time = QLabel(Dialog)
        self.time.setObjectName("label_4")
        self.time.setGeometry(QRect(270, 50, 91, 41))
        self.time.setStyleSheet(Copilot_st1)
        self.time.setFont(font)

        self.depth = QLabel(Dialog)
        self.depth.setObjectName(u"label_5")
        self.depth.setGeometry(QRect(540, 50, 101, 41))
        self.depth.setStyleSheet(Copilot_st1)
        self.depth.setFont(font)

        self.line_1 = QFrame(Dialog)
        self.line_1.setObjectName(u"line_2")
        self.line_1.setGeometry(QRect(210, 100, 511, 20))
        self.line_1.setFrameShape(QFrame.Shape.HLine)
        self.line_1.setFrameShadow(QFrame.Shadow.Sunken)

        self.back_button = QPushButton(Dialog)
        self.back_button.setObjectName("pushButton")
        self.back_button.setGeometry(QRect(10, 10, 61, 41))
        icon = QIcon.fromTheme("go-previous")
        self.back_button.setIcon(icon)
        self.back_button.setStyleSheet(back_st)
        self.back_button.setFont(QFont("Gill Sans",12))
        
        self.depth_labels = []
        self.current_index = 0
        
        y_position = 110
        for i in range(1, 11):
            time_var=1
            
            t_label = QLabel(Dialog)
            t_label.setObjectName(f"time label_{6 + 2 * i}")
            t_label.setGeometry(QRect(300, y_position, 31, 31))
            t_label.setStyleSheet(float_st) # Setting dynamic text (1s, 2s, etc.)
            t_label.setText(QCoreApplication.translate("Dialog", f"{time_var*i}s", None))

            d_label = QLabel(Dialog)
            d_label.setObjectName(f"depth label_{7 + 2 * i}")
            d_label.setGeometry(QRect(570, y_position, 31, 31))
            d_label.setStyleSheet(float_st)
            d_label.setText(QCoreApplication.translate("Dialog", "...", None))

            self.depth_labels.append(d_label)

            line = QFrame(Dialog)
            line.setObjectName(f"line_{i}")
            line.setGeometry(QRect(210, y_position + 30, 511, 20))
            line.setFrameShape(QFrame.Shape.HLine)
            line.setFrameShadow(QFrame.Shadow.Sunken)
            y_position += 40  

        self.setText(Dialog)
        QMetaObject.connectSlotsByName(Dialog)

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.Bg_label.setText("")
        self.Dlabel.setText("")
        self.DT.setText(QCoreApplication.translate("Dialog", "   Depth Tracker", None))
        self.time.setText(QCoreApplication.translate("Dialog", "Time", None))
        self.depth.setText(QCoreApplication.translate("Dialog","Depth ", None))
        self.back_button.setText(QCoreApplication.translate("Dialog", "Back", None))
    
    def update_depth(self, depth_value):
        
        if self.current_index < len(self.depth_labels):
            
            self.depth_labels[self.current_index].setText(f"{depth_value:.2f}m")
            self.current_index += 1
        else:   
            print("All labels have been updated.")