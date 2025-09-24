
# -*- coding: utf-8 -*-
# ui_mainwindow.py (updated with QSplitter, single-line DAC group, proper objectNames for QSS)

from PySide6.QtCore import (
    QCoreApplication,
    QMetaObject,
    Qt,
    QSize,
)
from PySide6.QtGui import QAction, QFont
from PySide6.QtWidgets import (
    QApplication,
    QAbstractButton,
    QButtonGroup,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenuBar,
    QPushButton,
    QRadioButton,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QSplitter,
)
from pyqtgraph import PlotWidget


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 720)

        # Actions
        self.actionExit = QAction(MainWindow)
        self.actionExit.setObjectName("actionExit")
        self.actionRecord = QAction(MainWindow)
        self.actionRecord.setObjectName("actionRecord")
        self.actionRecord.setCheckable(True)

        # Central and main vertical layout
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_main = QVBoxLayout(self.centralwidget)
        self.verticalLayout_main.setObjectName("verticalLayout_main")

        # --- Device Connection (top group) ---
        self.connection_group = QGroupBox(self.centralwidget)
        self.connection_group.setObjectName("connection_group")
        sizePolicy_conn = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.connection_group.setSizePolicy(sizePolicy_conn)
        self.connection_group.setMinimumHeight(84)

        self.horizontalLayout_connection = QHBoxLayout(self.connection_group)
        self.horizontalLayout_connection.setObjectName("horizontalLayout_connection")
        self.horizontalLayout_connection.setContentsMargins(9, 0, 9, 6)
        self.horizontalLayout_connection.setSpacing(8)

        self.label_port = QLabel(self.connection_group)
        self.label_port.setObjectName("label_port")
        self.port_combo = QComboBox(self.connection_group)
        self.port_combo.setObjectName("port_combo")
        self.port_combo.setMinimumSize(QSize(150, 0))
        self.refresh_button = QPushButton(self.connection_group)
        self.refresh_button.setObjectName("refresh_button")
        self.connect_button = QPushButton(self.connection_group)
        self.connect_button.setObjectName("connect_button")
        self.disconnect_button = QPushButton(self.connection_group)
        self.disconnect_button.setObjectName("disconnect_button")
        self.connection_status_label = QLabel(self.connection_group)
        self.connection_status_label.setObjectName("connection_status_label")
        self.connection_status_label.setMinimumSize(QSize(120, 0))
        self.connection_status_label.setText("")

        self.horizontalLayout_connection.addWidget(self.label_port)
        self.horizontalLayout_connection.addWidget(self.port_combo)
        self.horizontalLayout_connection.addWidget(self.refresh_button)
        self.horizontalLayout_connection.addWidget(self.connect_button)
        self.horizontalLayout_connection.addWidget(self.disconnect_button)
        self.horizontalLayout_connection.addWidget(self.connection_status_label)
        self.horizontalLayout_connection.addStretch(1)

        # Logo (top-right, slightly larger)
        self.logo_label = QLabel(self.connection_group)
        self.logo_label.setObjectName("logo_label")
        self.logo_label.setFixedSize(120, 80)
        self.logo_label.setAlignment(Qt.AlignRight | Qt.AlignTop)
        self.logo_label.setStyleSheet("margin-top: -10px;")
        self.horizontalLayout_connection.addWidget(self.logo_label, 0, Qt.AlignRight | Qt.AlignTop)
        self.horizontalLayout_connection.addWidget(self.logo_label)
        self.verticalLayout_main.addWidget(self.connection_group)

        # --- Splitter (Control Panel | Plots) ---
        self.splitter = QSplitter(Qt.Horizontal, self.centralwidget)
        self.splitter.setObjectName("splitter_main")
        self.splitter.setChildrenCollapsible(True)
        self.splitter.setHandleWidth(6)
        self.splitter.setSizes([350, 970])

        # Left: Control Panel group # parent is splitter
        self.control_panel_group = QGroupBox(self.splitter)  # parent is splitter
        self.control_panel_group.setObjectName("control_panel_group")
        sizePolicy_Control = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        self.control_panel_group.setMaximumWidth(470)
        self.control_panel_group.setSizePolicy(sizePolicy_Control)
        self.verticalLayout_controls = QVBoxLayout(self.control_panel_group)
        self.verticalLayout_controls.setObjectName("verticalLayout_controls")

        # --- Motor Control group ---
        self.motor_group = QGroupBox(self.control_panel_group)
        self.motor_group.setObjectName("motor_group")
        self.verticalLayout_motor = QVBoxLayout(self.motor_group)
        self.verticalLayout_motor.setObjectName("verticalLayout_motor")

        # Action buttons
        self.horizontalLayout_motor_action = QHBoxLayout()
        self.horizontalLayout_motor_action.setObjectName("horizontalLayout_motor_action")
        self.move_left_button = QPushButton(self.motor_group)
        self.move_left_button.setObjectName("move_left_button")
        self.move_right_button = QPushButton(self.motor_group)
        self.move_right_button.setObjectName("move_right_button")
        self.calibrate_button = QPushButton(self.motor_group)
        self.calibrate_button.setObjectName("calibrate_button")
        self.horizontalLayout_motor_action.addWidget(self.move_left_button)
        self.horizontalLayout_motor_action.addWidget(self.move_right_button)
        self.horizontalLayout_motor_action.addWidget(self.calibrate_button)
        self.verticalLayout_motor.addLayout(self.horizontalLayout_motor_action)

        # Speed
        self.speed_group = QGroupBox(self.motor_group)
        self.speed_group.setObjectName("speed_group")
        self.horizontalLayout_speed = QHBoxLayout(self.speed_group)
        self.horizontalLayout_speed.setObjectName("horizontalLayout_speed")
        self.radio_fast = QRadioButton(self.speed_group)
        self.radio_fast.setObjectName("radio_fast")
        self.radio_fast.setChecked(True)
        self.radio_smooth = QRadioButton(self.speed_group)
        self.radio_smooth.setObjectName("radio_smooth")
        self.radio_slow = QRadioButton(self.speed_group)
        self.radio_slow.setObjectName("radio_slow")
        self.horizontalLayout_speed.addStretch(1)
        self.horizontalLayout_speed.addWidget(self.radio_fast)
        self.horizontalLayout_speed.addSpacing(20)
        self.horizontalLayout_speed.addWidget(self.radio_smooth)
        self.horizontalLayout_speed.addSpacing(20)
        self.horizontalLayout_speed.addWidget(self.radio_slow)
        self.horizontalLayout_speed.addStretch(1)
        self.verticalLayout_motor.addWidget(self.speed_group)
        
        # ButtonGroup + IDs
        self.speed_button_group = QButtonGroup(MainWindow)
        self.speed_button_group.setObjectName("speed_button_group")
        self.speed_button_group.addButton(self.radio_fast, 1)
        self.speed_button_group.addButton(self.radio_smooth, 2)
        self.speed_button_group.addButton(self.radio_slow, 3)

        # Position label
        self.position_label = QLabel(self.motor_group)
        self.position_label.setObjectName("position_label")
        font_pos = QFont()
        font_pos.setPointSize(12)
        self.position_label.setFont(font_pos)
        self.position_label.setAlignment(Qt.AlignCenter)
        self.verticalLayout_motor.addWidget(self.position_label)

        # Manual Control group
        self.manual_group = QGroupBox(self.motor_group)
        self.manual_group.setObjectName("manual_group")
        self.verticalLayout_manual = QVBoxLayout(self.manual_group)
        self.verticalLayout_manual.setObjectName("verticalLayout_manual")

        self.horizontalLayout_manual_inputs = QHBoxLayout()
        self.horizontalLayout_manual_inputs.setObjectName("horizontalLayout_manual_inputs")
        self.label_start = QLabel(self.manual_group)
        self.label_start.setObjectName("label_start")
        self.start_pos_input = QLineEdit(self.manual_group)
        self.start_pos_input.setObjectName("start_pos_input")
        self.start_pos_input.setFixedWidth(76)
        self.label_end = QLabel(self.manual_group)
        self.label_end.setObjectName("label_end")
        self.end_pos_input = QLineEdit(self.manual_group)
        self.end_pos_input.setObjectName("end_pos_input")
        self.end_pos_input.setFixedWidth(76)
        self.label_cycle = QLabel(self.manual_group)
        self.label_cycle.setObjectName("label_cycle")
        self.cycle_spinbox = QSpinBox(self.manual_group)
        self.cycle_spinbox.setObjectName("cycle_spinbox")
        self.cycle_spinbox.setFixedWidth(70)
        self.cycle_spinbox.setRange(1, 9999)
        self.cycle_spinbox.setValue(1)
        self.horizontalLayout_manual_inputs.addWidget(self.label_start)
        self.horizontalLayout_manual_inputs.addWidget(self.start_pos_input)
        self.horizontalLayout_manual_inputs.addSpacing(20)
        self.horizontalLayout_manual_inputs.addWidget(self.label_end)
        self.horizontalLayout_manual_inputs.addWidget(self.end_pos_input)
        self.horizontalLayout_manual_inputs.addSpacing(20)
        self.horizontalLayout_manual_inputs.addWidget(self.label_cycle)
        self.horizontalLayout_manual_inputs.addWidget(self.cycle_spinbox)
        self.horizontalLayout_manual_inputs.addStretch(1)
        self.verticalLayout_manual.addLayout(self.horizontalLayout_manual_inputs)

        self.horizontalLayout_manual_buttons = QHBoxLayout()
        self.horizontalLayout_manual_buttons.setObjectName("horizontalLayout_manual_buttons")
        self.manual_run_button = QPushButton(self.manual_group)
        self.manual_run_button.setObjectName("manual_run_button")
        self.manual_pause_button = QPushButton(self.manual_group)
        self.manual_run_button.setMinimumWidth(60)
        self.manual_pause_button.setObjectName("manual_pause_button")
        self.manual_full_stop_button = QPushButton(self.manual_group)
        self.manual_full_stop_button.setObjectName("manual_full_stop_button")
        self.manual_cycle_label = QLabel(self.manual_group)
        self.manual_cycle_label.setObjectName("manual_cycle_label")
        
        self.horizontalLayout_manual_buttons.addWidget(self.manual_run_button)
        self.horizontalLayout_manual_buttons.addWidget(self.manual_pause_button)
        self.horizontalLayout_manual_buttons.addWidget(self.manual_full_stop_button)
        self.horizontalLayout_manual_buttons.addSpacing(65)
        self.horizontalLayout_manual_buttons.addWidget(self.manual_cycle_label)
        self.horizontalLayout_manual_buttons.addStretch(1)
        self.verticalLayout_manual.addLayout(self.horizontalLayout_manual_buttons)
        

        self.verticalLayout_motor.addWidget(self.manual_group)
        self.verticalLayout_controls.addWidget(self.motor_group)
        

        # --- Reference Voltage (single-line layout) ---
        self.dac_group = QGroupBox(self.control_panel_group)
        self.dac_group.setObjectName("dac_group")
        self.horizontalLayout_dac = QHBoxLayout(self.dac_group)
        self.horizontalLayout_dac.setObjectName("horizontalLayout_dac")

        self.label_dac_set = QLabel(self.dac_group)
        self.label_dac_set.setObjectName("label_dac_set")
        self.dac_voltage_input = QLineEdit(self.dac_group)
        self.dac_voltage_input.setObjectName("dac_voltage_input")
        self.dac_voltage_input.setFixedWidth(70)
        self.label_dac_current = QLabel(self.dac_group)
        self.label_dac_current.setObjectName("label_dac_current")
        self.dac_current_label = QLabel(self.dac_group)
        self.dac_current_label.setObjectName("dac_current_label")
        self.dac_set_button = QPushButton(self.dac_group)
        self.dac_set_button.setObjectName("dac_set_button")
        self.dac_set_button.setMinimumWidth(80)

        self.horizontalLayout_dac.addWidget(self.label_dac_set)
        self.horizontalLayout_dac.addSpacing(8)
        self.horizontalLayout_dac.addWidget(self.dac_voltage_input)
        self.horizontalLayout_dac.addSpacing(10)
        self.horizontalLayout_dac.addWidget(self.dac_set_button)
        self.horizontalLayout_dac.addSpacing(20) # فاصله بیشتر
        self.horizontalLayout_dac.addWidget(self.label_dac_current)
        self.horizontalLayout_dac.addWidget(self.dac_current_label)
        self.horizontalLayout_dac.addStretch(1)

        self.verticalLayout_controls.addWidget(self.dac_group)

        # --- Resistance Select ---
        self.res_group = QGroupBox(self.control_panel_group)
        self.res_group.setObjectName("res_group")
        self.horizontalLayout_res = QHBoxLayout(self.res_group)
        self.horizontalLayout_res.setObjectName("horizontalLayout_res")

        self.label_res = QLabel(self.res_group)
        self.label_res.setObjectName("label_res")
        self.res_slider = QSlider(Qt.Horizontal, self.res_group)
        self.res_slider.setObjectName("res_slider")
        self.res_slider.setRange(0, 15)
        self.res_slider.setTickPosition(QSlider.TicksBelow)
        self.res_slider.setTickInterval(1)
        self.res_slider.setSingleStep(1)
        self.res_slider.setPageStep(1)
        self.res_slider.setTracking(True)
        sizePolicy_slider = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.res_slider.setSizePolicy(sizePolicy_slider)
        self.res_slider.setMinimumWidth(260)
        self.res_value_label = QLabel(self.res_group)
        self.res_value_label.setObjectName("res_value_label")

        self.horizontalLayout_res.addWidget(self.label_res)
        self.horizontalLayout_res.addWidget(self.res_slider)
        self.horizontalLayout_res.addWidget(self.res_value_label)
        self.horizontalLayout_res.setStretch(0, 0)
        self.horizontalLayout_res.setStretch(1, 1)
        self.horizontalLayout_res.setStretch(2, 0)

        self.verticalLayout_controls.addWidget(self.res_group)

        # --- Log ---
        self.label_log = QLabel(self.control_panel_group)
        self.label_log.setObjectName("label_log")
        self.log_text_edit = QTextEdit(self.control_panel_group)
        self.log_text_edit.setObjectName("log_text_edit")
        self.log_text_edit.setReadOnly(True)
        self.verticalLayout_controls.addWidget(self.label_log)
        self.verticalLayout_controls.addWidget(self.log_text_edit)

        self.verticalLayout_controls.addStretch(1)

        # Add control panel group to splitter (already parented)
        self.splitter.addWidget(self.control_panel_group)

        # Right: Data Panel Tabs
        self.data_panel_tabs = QTabWidget(self.splitter)  # parent is splitter
        self.data_panel_tabs.setObjectName("data_panel_tabs")
        sizePolicy_Data = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.data_panel_tabs.setSizePolicy(sizePolicy_Data)
        self.data_panel_tabs.setMinimumWidth(640) # <--- (جلوگیری از کوچک شدن بیش از حد)

        # Tab 1: Voltage vs Time
        self.tab_adc = QWidget()
        self.tab_adc.setObjectName("tab_adc")
        self.verticalLayout_adc = QVBoxLayout(self.tab_adc)
        self.verticalLayout_adc.setObjectName("verticalLayout_adc")
        self.verticalLayout_adc.setContentsMargins(0, 0, 0, 0)
        self.adc_plot_widget = PlotWidget(self.tab_adc)
        self.adc_plot_widget.setObjectName("adc_plot_widget")
        self.verticalLayout_adc.addWidget(self.adc_plot_widget)
        self.data_panel_tabs.addTab(self.tab_adc, "")

        # Tab 2: Resistance vs Time
        self.tab_resistance = QWidget()
        self.tab_resistance.setObjectName("tab_resistance")
        self.verticalLayout_res = QVBoxLayout(self.tab_resistance)
        self.verticalLayout_res.setObjectName("verticalLayout_res")
        self.verticalLayout_res.setContentsMargins(0, 0, 0, 0)
        self.res_plot_widget = PlotWidget(self.tab_resistance)
        self.res_plot_widget.setObjectName("res_plot_widget")
        self.verticalLayout_res.addWidget(self.res_plot_widget)
        self.data_panel_tabs.addTab(self.tab_resistance, "")

        # Tab 3: Voltage vs Disp.
        self.tab_v_vs_d = QWidget()
        self.tab_v_vs_d.setObjectName("tab_v_vs_d")
        self.verticalLayout_v_vs_d = QVBoxLayout(self.tab_v_vs_d)
        self.verticalLayout_v_vs_d.setObjectName("verticalLayout_v_vs_d")
        self.verticalLayout_v_vs_d.setContentsMargins(0, 0, 0, 0)
        self.v_vs_d_plot_widget = PlotWidget(self.tab_v_vs_d)
        self.v_vs_d_plot_widget.setObjectName("v_vs_d_plot_widget")
        self.verticalLayout_v_vs_d.addWidget(self.v_vs_d_plot_widget)
        self.data_panel_tabs.addTab(self.tab_v_vs_d, "")

        # Tab 4: Resistance vs Disp.
        self.tab_r_vs_d = QWidget()
        self.tab_r_vs_d.setObjectName("tab_r_vs_d")
        self.verticalLayout_r_vs_d = QVBoxLayout(self.tab_r_vs_d)
        self.verticalLayout_r_vs_d.setObjectName("verticalLayout_r_vs_d")
        self.verticalLayout_r_vs_d.setContentsMargins(0, 0, 0, 0)
        self.r_vs_d_plot_widget = PlotWidget(self.tab_r_vs_d)
        self.r_vs_d_plot_widget.setObjectName("r_vs_d_plot_widget")
        self.verticalLayout_r_vs_d.addWidget(self.r_vs_d_plot_widget)
        self.data_panel_tabs.addTab(self.tab_r_vs_d, "")

        # Add tabs to splitter
        self.splitter.addWidget(self.data_panel_tabs)

        # Stretch factors (left:1, right:4)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 4)

        # Add splitter to main vertical layout
        self.verticalLayout_main.addWidget(self.splitter)

        # Set main layout stretches: connection bar small, content fills
        self.verticalLayout_main.setStretch(0, 0)  # connection_group
        self.verticalLayout_main.setStretch(1, 1)  # splitter
        
        

        MainWindow.setCentralWidget(self.centralwidget)

        # Menu and status bar
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName("menubar")
        self.menubar.setGeometry(0, 0, 1280, 22)
        self.menuFile = self.menubar.addMenu("")
        self.menuFile.setObjectName("menuFile")
        self.menuFile.addAction(self.actionExit)
        self.menubar.addAction(self.actionRecord)
        MainWindow.setMenuBar(self.menubar)

        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _t = QCoreApplication.translate
        MainWindow.setWindowTitle(_t("MainWindow", u"Resistance Meter Program", None))

        # Menus and actions
        self.actionExit.setText(_t("MainWindow", u"E&xit", None))
        self.actionExit.setShortcut(_t("MainWindow", u"Ctrl+Q", None))
        self.actionRecord.setText(_t("MainWindow", u"&Record Data", None))
        self.menuFile.setTitle(_t("MainWindow", u"&File", None))

        # Connection Group
        self.connection_group.setTitle(_t("MainWindow", u"Device Connection", None))
        self.label_port.setText(_t("MainWindow", u"Serial Port:", None))
        self.refresh_button.setText(_t("MainWindow", u"Refresh", None))
        self.connect_button.setText(_t("MainWindow", u"Connect", None))
        self.disconnect_button.setText(_t("MainWindow", u"Disconnect", None))
        # connection_status_label is dynamic

        # Control Panel
        self.control_panel_group.setTitle(_t("MainWindow", u"Control Panel", None))

        # Motor Control
        self.motor_group.setTitle(_t("MainWindow", u"Motor Control", None))
        self.move_left_button.setText(_t("MainWindow", u" Left ", None))
        self.move_right_button.setText(_t("MainWindow", u" Right ", None))
        self.calibrate_button.setText(_t("MainWindow", u"Calibrate Motor", None))

        self.speed_group.setTitle(_t("MainWindow", u"Speed", None))
        self.radio_fast.setText(_t("MainWindow", u"Fast", None))
        self.radio_smooth.setText(_t("MainWindow", u"Smooth", None))
        self.radio_slow.setText(_t("MainWindow", u"Slow", None))

        self.position_label.setText(_t("MainWindow", u"Position: 0.000 cm | --- cm | --- cm", None))

        # Manual Control
        self.manual_group.setTitle(_t("MainWindow", u"Manual Control <b>(cm)<b>", None))
        self.label_start.setText(_t("MainWindow", u"Start :", None))
        self.start_pos_input.setPlaceholderText(_t("MainWindow", u" 1.500 ", None))
        self.label_end.setText(_t("MainWindow", u"End :", None))
        self.end_pos_input.setPlaceholderText(_t("MainWindow", u" 10.250 ", None))
        self.label_cycle.setText(_t("MainWindow", u"Cycles:", None))
        self.manual_run_button.setText(_t("MainWindow", u"Run", None))
        self.manual_pause_button.setText(_t("MainWindow", u"Pause", None))
        self.manual_full_stop_button.setText(_t("MainWindow", u"STOP", None))
        self.manual_cycle_label.setText(_t("MainWindow", u"<b>Cycles Done:</b> 0", None))

        # DAC (Reference Voltage)
        self.dac_group.setTitle(_t("MainWindow", u"Reference Voltage", None))
        self.label_dac_set.setText(_t("MainWindow", u"Set Voltage:", None))
        self.dac_voltage_input.setPlaceholderText(_t("MainWindow", u"0.1V:4.0V", None))
        self.label_dac_current.setText(_t("MainWindow", u"<b>Voltage set:<b>", None))
        # dac_current_label value set in code
        self.dac_set_button.setText(_t("MainWindow", u"Set", None))

        # Resistance Select
        self.res_group.setTitle(_t("MainWindow", u"Resistance Select", None))
        self.label_res.setText(_t("MainWindow", u"Channel:", None))
        self.res_value_label.setText(_t("MainWindow", u" 3.9 kΩ", None))

        # Log
        self.label_log.setText(_t("MainWindow", u"Log:", None))

        # Tabs
        self.data_panel_tabs.setTabText(self.data_panel_tabs.indexOf(self.tab_adc), _t("MainWindow", u"Voltage Vs Time", None))
        self.data_panel_tabs.setTabText(self.data_panel_tabs.indexOf(self.tab_resistance), _t("MainWindow", u"Resistance Vs Time", None))
        self.data_panel_tabs.setTabText(self.data_panel_tabs.indexOf(self.tab_v_vs_d), _t("MainWindow", u"Voltage vs Disp.", None))
        self.data_panel_tabs.setTabText(self.data_panel_tabs.indexOf(self.tab_r_vs_d), _t("MainWindow", u"Resistance vs Disp.", None))