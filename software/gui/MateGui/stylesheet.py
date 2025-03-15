
#This is the CSS code for each stylesheet i made & used

Laning_buttons_st = """
QPushButton {
    background-color: #2980B9;
    color: white;  
    border-radius: 10px; 
    border: 2px solid #3498DB; 
    padding: 10px 20px; 

}

QPushButton:hover {
    background-color: #1D6FA0;
    border-color: #5DADE2; 
}

QPushButton:pressed {
    background-color: #1B5672;
    border-color: #3498DB; 
}
"""
Engineer_buttons_st= """
QPushButton {
    background-color: #2980B9;
    color: white; 
    border-radius: 10px; 
    border: 2px solid #3498DB; 
    font-family: "Gill Sans", sans-serif;

    padding: 12px 24px;  

}

QPushButton:hover {
    background-color: #1D6FA0;  
    border-color: #5DADE2;  
}

QPushButton:pressed {
    background-color: #1B5672;  
    border-color: #3498DB; 
}
"""
Copilot_st1 = """
QLabel {
    border: 2px solid rgb(37, 39, 48);
    border-radius: 20px;
    color: #FFF;
    padding-left: 20px;
    padding-right: 20px;
    background-color: #2980B9;
    font-family: "Gill Sans", sans-serif;

}

QLabel:hover {
    border: 2px solid rgb(48, 50, 62);
}

QLabel:focus {
    border: 2px solid rgb(85, 170, 255);
    background-color: rgb(43, 45, 56);
}
"""
Copilot_st2= """
QLabel {
    border: 2px solid rgb(37, 39, 48);
    border-radius: 20px;
    color: #FFF;
    padding-left: 20px;
    padding-right: 20px;
    background-color: rgb(34, 36, 44);
    font-family: "Gill Sans", sans-serif;

}

QLabel:hover {
    border: 2px solid rgb(48, 50, 62);
}

QLabel:focus {
    border: 2px solid rgb(85, 170, 255);
    background-color: rgb(43, 45, 56);
}
"""
red_button = """
QPushButton {
    background-color: #D45D5D;  
    color: white; 
    border-radius: 10px; 
    border: 2px solid #B03A2E;
    font-family: "Gill Sans", sans-serif;

    padding: 12px 24px; 

}

QPushButton:hover {
    background-color: #B94A3A; 
    border-color: #A04030;  
}

QPushButton:pressed {
    background-color: #9C3636;  
    border-color: #B03A2E;  
}
"""
apply_st="""            QPushButton {
                background-color: rgba(0, 0, 0, 0.2);
                border: none;
                border-radius: 8px;
                padding: 6px;
                font-family: "Gill Sans", sans-serif;
                color: white;
            }
            QPushButton:hover, QPushButton:pressed {
                background-color: rgba(0, 0, 0, 0.2);
            }"""

back_st="""
    QPushButton {
        background-color: rgba(0, 0, 0, 0.2);
        border: none;
        border-radius: 8px;
        padding: 6px;
        font-family: "Gill Sans", sans-serif;
        color: white;
    }
    QPushButton:hover, QPushButton:pressed {
        background-color: rgba(0, 0, 0, 0.2);
    }
"""

selection_st="""    QComboBox {
        background-color: rgba(0, 0, 0, 0.2);
        border: 1px solid rgba(255, 255, 255, 0.3);
        border-radius: 6px;
        padding: 5px 30px 5px 10px;
        font-family: "Gill Sans", sans-serif;
        color: white;
        selection-background-color: rgba(255, 255, 255, 0.2);
    }
    
    QComboBox::drop-down {
        subcontrol-origin: padding;
        subcontrol-position: right center;
        width: 25px;
        border: none;
        background: transparent;
    }
    
    QComboBox::down-arrow {
        image: none;
        width: 0; 
        height: 0;
        border-left: 5px solid transparent;
        border-right: 5px solid transparent;
        border-top: 7px solid rgba(255, 255, 255, 0.8);
    }
    

    
    QComboBox:hover {
        background-color: rgba(0, 0, 0, 0.25);
    }
    
    /* Keep the previous item view styling */
    QComboBox QAbstractItemView {
        background-color: rgba(0, 0, 0, 0.3);
        border: 1px solid rgba(255, 255, 255, 0.2);
        border-radius: 6px;
        color: white;
        font-family: "Gill Sans", sans-serif;
        selection-background-color: rgba(255, 255, 255, 0.2);
        outline: none;
    }"""
float_st="""QLabel {
    background-color: rgba(0, 0, 0, 0.2);
    border: none;
    border-radius: 8px;
    padding: 6px;
    font-family: "Gill Sans", sans-serif;
    color: white;
    text-align: center; /* Center the text horizontally */
    vertical-align: middle; /* Center the text vertically */
}

QPushButton:hover, QPushButton:pressed {
    background-color: rgba(0, 0, 0, 0.2);
}
"""