# main_app.py (v11.0 – Manual control extracted to manual_control.py)
# Main application logic for the Linear Rail Controller.

import sys
import os
import re
import time

from PySide6.QtWidgets import (
    QApplication, QStyle, QMainWindow, QMessageBox, QFileDialog, QLabel
)
from PySide6.QtCore import Qt, Slot, QTimer, QThread, Signal, QObject, QSize
from PySide6.QtGui import QValidator, QIcon, QDoubleValidator, QPixmap

from ui_mainwindow import Ui_MainWindow
from serial_handler import SerialHandler
from plot_handler import PlotHandler
from manual_control import ManualController

import pyqtgraph as pg

# ---------------- Constants ----------------
RES_VALUES = [3.9, 5.6, 6.8, 10.0, 15.0, 22.0, 27.0, 33.0,
              39.0, 47.0, 56.0, 68.0, 82.0, 100.0, 120.0, 150.0]
CM_PER_FULL_STEP = 0.001
MM_PER_FULL_STEP = 0.01  # 1 full step = 0.01 mm
ADC_VOLTAGE_RANGE_IN_PYTHON = 4.096
ADC_RESOLUTION_IN_PYTHON = 32768.0


# ===================================================================
# Data Processor (Worker QObject in a separate QThread)
# ===================================================================
class DataProcessor(QObject):
    data_ready_for_ui = Signal(dict)

    def __init__(self, main_window_ref):
        super().__init__()
        self.main_window = main_window_ref

        # فیلتر ولتاژ بهبود یافته
        self.adc_deadband_v = 0.005     # 5mV
        self.adc_iir_alpha = 0.7        # IIR (0.2..0.4 نرم‌تر)
        self.v_last_f = None
        self.mux_change_detected = False
        self.last_Rt_k = None
        self.post_mux_drop_n = 0
        # Short resistance-mute window to suppress spikes around DAC/MUX changes (seconds)
        self.rt_mute_window_s = 0.1
        self.rt_mute_until = 0.0
        self._last_v_for_plot = None  # جهت استفاده در hold

    @Slot(str)
    def process_raw_data(self, data: str):
        output_data = {}
        try:
            s = data.strip()
            # Arm short resistance-mute window when DAC changes (from device logs)
            if s.startswith("-> SET_DAC_V") or s.startswith("DAC set"):
                self.rt_mute_until = time.time() + self.rt_mute_window_s
            Vi = getattr(self.main_window, 'current_dac_voltage', 4.096)
            if Vi < 0.05: Vi = 0.05  # حداقل ایمن
            Rm = float(self.main_window.current_selected_res_kohm) if hasattr(self.main_window, 'current_selected_res_kohm') else 0.0

            if s == "$PAUSED":
                output_data['manual_run_paused'] = True

            elif s.startswith("ADC:"):
                parts_adc = s.split(":")
                v_in = float(parts_adc[1].strip()) if len(parts_adc) >= 2 else 0.0
                now = time.time()
                v = self._filter_voltage(v_in, now)
                # Gate states
                mux_mute = hasattr(self.main_window, 'mux_mute_until') and (now < self.main_window.mux_mute_until)
                rt_mute  = (now < getattr(self, 'rt_mute_until', 0.0))
                if self.post_mux_drop_n > 0 and not mux_mute:
                    self.post_mux_drop_n -= 1

                # Optional Rt from device (unused currently; placeholder)
                rt_from_device = None
                try:
                    if len(parts_adc) >= 3:
                        rt_from_device = float(parts_adc[2].strip())
                except Exception:
                    rt_from_device = None

                if rt_from_device is not None:
                    Rt = rt_from_device
                else:
                    # انتخاب Rm مؤثر و محاسبه محلی
                    Rm_eff = self._effective_Rm(now)
                    den = (Vi - v)
                    if den > 1e-6 and Rm_eff > 0:
                        Rt = (v * Rm_eff) / den
                    else:
                        Rt = self.last_Rt_k if self.last_Rt_k is not None else 0.0

                # Hold previous Rt briefly after DAC/MUX changes
                if rt_mute and self.last_Rt_k is not None:
                    Rt_to_plot = self.last_Rt_k
                else:
                    Rt_to_plot = Rt
                    self.last_Rt_k = Rt_to_plot

                output_data['live_plot_point'] = (v, Rt_to_plot)
                output_data['latest_adc_voltage'] = v

            elif s.startswith("D:"):
                try:
                    parts = s[2:].strip().split(':')
                    if len(parts) >= 2:
                        pos_from_arduino = int(parts[0].strip())
                        val_str = parts[1].strip()
                        if ('.' in val_str) or ('e' in val_str.lower()):
                            v_in = float(val_str)
                        else:
                            raw_adc = int(val_str)
                            v_in = (float(raw_adc) / ADC_RESOLUTION_IN_PYTHON) * ADC_VOLTAGE_RANGE_IN_PYTHON

                        now = time.time()
                        v = self._filter_voltage(v_in, now)

                        # Gate states
                        mux_mute = hasattr(self.main_window, 'mux_mute_until') and (now < self.main_window.mux_mute_until)
                        rt_mute  = (now < getattr(self, 'rt_mute_until', 0.0))

                        if self.post_mux_drop_n > 0 and not mux_mute:
                            self.post_mux_drop_n -= 1

                        rt_from_device = None
                        try:
                            if len(parts) >= 3:
                                rt_from_device = float(parts[2].strip())
                        except Exception:
                            rt_from_device = None

                        Vi = getattr(self.main_window, 'current_dac_voltage', 4.096)
                        if Vi < 0.05: Vi = 0.05
                        if rt_from_device is not None:
                            Rt = rt_from_device
                        else:
                            Rm_eff = self._effective_Rm(now)
                            den = (Vi - v)
                            if den > 1e-6 and Rm_eff > 0:
                                Rt = (v * Rm_eff) / den
                            else:
                                Rt = self.last_Rt_k if self.last_Rt_k is not None else 0.0

                        # Short resistance mute window (hold Rt) — DAC/MUX deglitch
                        if (rt_mute or mux_mute or self.post_mux_drop_n > 0) and self.last_Rt_k is not None:
                            Rt_to_plot = self.last_Rt_k
                            v_to_plot  = self._last_v_for_plot if self._last_v_for_plot is not None else v
                        else:
                            Rt_to_plot = Rt
                            v_to_plot  = v
                            self.last_Rt_k = Rt_to_plot
                            self._last_v_for_plot = v_to_plot

                        # Time plots
                        output_data['live_plot_point'] = (v, Rt_to_plot)
                        output_data['position_update'] = pos_from_arduino
                        output_data['latest_adc_voltage'] = v

                        # Displacement plots: emit ONLY when fully stable
                        if self.main_window.is_manual_running:
                            mf = max(1, int(getattr(self.main_window, 'microstep_factor', 1)))
                            current_pos_full = pos_from_arduino / mf
                            leg_start_full = getattr(self.main_window, 'current_leg_start_abs_microsteps', 0) / mf
                            total_leg_dist_full_steps = abs(
                                getattr(self.main_window, 'current_manual_run_end_abs_steps', 0) -
                                getattr(self.main_window, 'current_manual_run_start_abs_steps', 0)
                            )
                            dist_from_leg_start_full = abs(current_pos_full - leg_start_full)
                            going_to_end = bool(getattr(self.main_window, 'current_leg_is_going_to_end', True))
                            if going_to_end:
                                relative_distance_mm = dist_from_leg_start_full * MM_PER_FULL_STEP
                            else:
                                relative_distance_mm = (total_leg_dist_full_steps - dist_from_leg_start_full) * MM_PER_FULL_STEP

                            max_leg_dist_mm = total_leg_dist_full_steps * MM_PER_FULL_STEP
                            if relative_distance_mm < 0.0: relative_distance_mm = 0.0
                            if max_leg_dist_mm > 0 and relative_distance_mm > max_leg_dist_mm:
                                relative_distance_mm = max_leg_dist_mm

                            output_data['displacement_plot_point'] = (relative_distance_mm, v_to_plot, Rt_to_plot)

                            # ضبط CSV: اگر نمی‌خواهی نمونه‌های hold ذخیره شوند، شرط بگذار!
                            if getattr(self.main_window, 'is_actively_recording', False):
                                cycle_num = getattr(self.main_window, 'manual_completed_cycles', 0) + 1
                                relative_time_sec = time.time() - float(getattr(self.main_window, 'manual_run_start_time', now))
                                data_row = (
                                    cycle_num,
                                    f"{relative_time_sec:.3f}",
                                    f"{relative_distance_mm:.2f}",
                                    f"{v_to_plot:.4f}",
                                    f"{Rt_to_plot:.3f}"
                                )
                                output_data['record_row'] = data_row
                    else:
                        output_data['log_message'] = f"Warning: Malformed D message: {s}"
                except Exception as e:
                    output_data['log_message'] = f"Error parsing D message: {e} | {s}"

            elif s.startswith("TOTAL_STEPS:"):
                try:
                    steps_str = s[len("TOTAL_STEPS:"):].strip()
                    filtered = re.sub(r'[^\d-]', '', steps_str)
                    if filtered:
                        total_steps = int(filtered)
                        output_data['total_steps_update'] = total_steps
                except Exception as e:
                    output_data['log_message'] = f"Error parsing TOTAL_STEPS: {e}"

            elif s == "$CALIBRATION_DONE":
                output_data['calibration_done'] = True

            elif s.startswith("CYCLE_DONE:"):
                try:
                    n = int(re.sub(r'[^\d-]', '', s.split(":", 1)[1].strip()))
                    output_data['cycle_done'] = n
                except Exception as e:
                    output_data['log_message'] = f"Error parsing CYCLE_DONE: {e}"

            elif s.lower().startswith("manual target leg reached"):
                output_data['manual_target_reached'] = True

            elif s.lower().startswith("manual run paused"):
                output_data['manual_run_paused'] = True

            elif s.startswith("Starting Calibration Process..."):
                output_data['calibration_started'] = True
            elif s.lower().startswith("all manual cycles completed"):
                output_data['all_manual_cycles_completed'] = True

            elif s.startswith("POSITION:"):
                try:
                    pos_str_clean = re.sub(r'[^\d-]', '', s.split(":",1)[1]).strip()
                    if pos_str_clean:
                        output_data['position_update'] = int(pos_str_clean)
                except Exception as e:
                    output_data['log_message'] = f"Error parsing POSITION: {e}"
            elif s.startswith("MUX_PINS:"):
                # فرمت: MUX_PINS:s0,s1,s2,s3 (S0 = LSB)
                try:
                    bits_str = s[len("MUX_PINS:"):].strip()
                    parts = [p.strip() for p in bits_str.split(",")]
                    if len(parts) == 4:
                        s0 = int(parts[0]); s1 = int(parts[1]); s2 = int(parts[2]); s3 = int(parts[3])
                        output_data['mux_pins'] = (s0, s1, s2, s3)
                        # محاسبه شماره کانال واقعی طبق سیم‌کشی آردوینو
                        ch = (s0 & 1) | ((s1 & 1) << 1) | ((s2 & 1) << 2) | ((s3 & 1) << 3)
                        output_data['mux_channel_confirmed'] = ch
                        # تشخیص تغییر کانال مالتی‌پلکسر برای فیلتر
                        self.mux_change_detected = True
                        self.post_mux_drop_n = 3
                        # Arm short Rt mute window on MUX change
                        self.rt_mute_until = time.time() + self.rt_mute_window_s
                        output_data['log_message'] = f"MUX channel CONFIRMED: {ch} (bits {s3}{s2}{s1}{s0})"
                except Exception as e:
                    output_data['log_message'] = f"Error parsing MUX_PINS: {e}"
            else:
                output_data['raw_message_for_log'] = s

        except Exception as e:
            output_data = {'log_message': f"Unexpected error processing: {e} | raw='{data}'"}

        if output_data:
            self.data_ready_for_ui.emit(output_data)

    def _effective_Rm(self, now):
        Rm_curr = float(getattr(self.main_window, 'current_selected_res_kohm', 0.0) or 0.0)
        Rm_prev = float(getattr(self.main_window, 'prev_selected_res_kohm', Rm_curr) or Rm_curr)
        mux_mute = hasattr(self.main_window, 'mux_mute_until') and (now < self.main_window.mux_mute_until)
        if mux_mute or (self.post_mux_drop_n > 0):
            return Rm_prev
        return Rm_curr

    def _reject_outlier(self, Rt):
        if self.last_Rt_k is None: return Rt
        if abs(Rt - self.last_Rt_k) > max(0.5, 0.10 * self.last_Rt_k):
            return self.last_Rt_k
        return Rt

    def _apply_tolerance(self, Rt: float) -> float:
        try:
            r = max(0.0, float(Rt))
            if r == 0.0:
                return 0.0
            base = [1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2]
            import math
            exp = int(math.floor(math.log10(r))) if r > 0 else 0
            candidates = []
            for d in range(exp - 1, exp + 2):
                scale = 10 ** d
                for b in base:
                    candidates.append(b * scale)
            nearest = min(candidates, key=lambda n: abs(n - r)) if candidates else r
            if nearest > 0 and abs(r - nearest) <= 0.05 * nearest:
                return round(nearest, 2)
            return round(r, 2)
        except Exception:
            try:
                return round(float(Rt), 2)
            except Exception:
                return 0.0

    def _filter_voltage(self, v_in: float, now: float) -> float:
        mux_mute = hasattr(self.main_window, 'mux_mute_until') and (now < self.main_window.mux_mute_until)
        # هنگام MUX-Mute مقدار قبلی را نگه می‌داریم
        if self.v_last_f is not None and mux_mute:
            return self.v_last_f

        # اگر تازه کانال MUX عوض شده، فیلتر را ریست کن و مقدار جدید را بلافاصله برگردان
        if self.mux_change_detected:
            self.v_last_f = v_in
            self.mux_change_detected = False
            return v_in

        # بدون مقدار قبلی؟ همان اول v_in را بپذیر
        if self.v_last_f is None:
            self.v_last_f = v_in
            return v_in

        # اگر جهش بزرگ است (مثل تغییر DAC)، فیلتر را بای‌پس کن → نمایش فوری
        change_magnitude = abs(v_in - self.v_last_f)
        if change_magnitude > 0.05:  # 50mV
            self.v_last_f = v_in
            return v_in

        # فیلتر نرم برای نویز کوچک
        if change_magnitude < self.adc_deadband_v:
            return self.v_last_f
        a = self.adc_iir_alpha
        v_f = (1.0 - a) * self.v_last_f + a * v_in
        self.v_last_f = v_f
        return v_f


# ===================================================================
# Main Window
# ===================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # UI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        style = QApplication.style()
        self.ui.move_left_button.setIcon(style.standardIcon(QStyle.SP_ArrowLeft))
        self.ui.move_left_button.setIconSize(QSize(16, 16))
        self.ui.move_left_button.setLayoutDirection(Qt.LeftToRight)
        self.ui.move_right_button.setIcon(style.standardIcon(QStyle.SP_ArrowRight))
        self.ui.move_right_button.setIconSize(QSize(16, 16))
        self.ui.move_right_button.setLayoutDirection(Qt.RightToLeft)
        self._set_conn_state_border("default")
        # 1. بارگذاری و نمایش لوگو
        try:
            # ساخت مسیر کامل به فایل لوگو در کنار فایل اجرایی
            current_dir = os.path.dirname(os.path.abspath(__file__))
            logo_path = os.path.join(current_dir, "logo.png")
            if os.path.exists(logo_path):
                pixmap = QPixmap(logo_path)
                self.ui.logo_label.setPixmap(pixmap.scaled(
                    self.ui.logo_label.width(),
                    self.ui.logo_label.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                ))
            else:
                self._log_message("Warning: logo.png not found.")
        except Exception as e:
            self._log_message(f"Error loading logo: {e}")
        # State
        self.mux_mute_until = 0.0

        self.is_calibrated = False
        self.current_motor_position = 0
        self.total_motor_steps = 0
        self.microstep_factor = 1
        self.current_dac_voltage = 1.0
        self.prev_selected_res_kohm = RES_VALUES[0]
        self.prev_mux_channel = 0
        self.current_mux_channel = 0
        self.current_selected_res_kohm = RES_VALUES[0]
        if hasattr(self.ui, 'res_slider'):
            self.ui.res_slider.setValue(0)
        if hasattr(self.ui, 'res_value_label'):
            self.ui.res_value_label.setText(f"{RES_VALUES[0]} kΩ")
        self.current_selected_res_kohm = RES_VALUES[0]

        # Manual run (اضافی: این فیلدها توسط ManualController هم نگهداری می‌شوند)
        self.is_manual_running = False
        self.is_manual_paused = False
        self.manual_completed_cycles = 0
        self.manual_run_original_start_cm = 0.0
        self.manual_run_original_end_cm = 0.0
        self.manual_run_original_cycles = 1
        self.current_manual_run_start_abs_steps = 0
        self.current_manual_run_end_abs_steps = 0
        self.current_leg_start_abs_microsteps = 0
        self.current_leg_is_going_to_end = True
        self.current_manual_leg_max_dist_mm = 0.0

        # Recording (نمایش وضعیت در status bar)
        self.is_recording_armed = False
        self.is_actively_recording = False
        self.recording_filepath = ""
        self.recorded_data_buffer = []
        self.manual_run_start_time = 0
        self.paused_relative_time = 0

        # ADC logging
        self.last_logged_adc_voltage = None
        self.adc_log_threshold = 0.01

        # Timers
        self.stop_flash_timer = QTimer(self)
        self.stop_flash_timer.setInterval(500)
        self.stop_button_flash_state = False

        self.plot_update_timer = QTimer(self)
        self.plot_update_interval = 16  # ms
        self.plot_update_timer.setInterval(self.plot_update_interval)

        # Serial
        self.serial_handler = SerialHandler(baud_rate=250000)

        # Data processor worker (QObject) in its own thread
        self.data_processor_thread = QThread()
        self.data_processor = DataProcessor(main_window_ref=self)
        self.data_processor.moveToThread(self.data_processor_thread)
        self.data_processor_thread.start()

        # Plot handler (centralized plotting)
        self.plot = PlotHandler(
            ui=self.ui,
            plot_time_window=30.0,
            max_time_points=10000,
            keep_last_n_cycles=4
        )
        self.plot.attach_hover_handlers()

        # Manual controller (extracted logic)
        self.manual = ManualController(
            main_window=self,
            serial_handler=self.serial_handler,
            plot_handler=self.plot,
            data_processor=self.data_processor
        )

        # UI setup
        self._configure_actions()
        self._configure_status_bar()
        self._setup_validators()
        self._connect_signals()
        self.manual.connect_ui_signals()  # اتصال‌های Manual/Recording

        # Initial UI state
        self.serial_handler.list_ports()
        self._update_controls_state()
        self._update_position_display()
        self.ui.dac_current_label.setText(f"{self.current_dac_voltage:.1f} V")
        self.plot_update_timer.start()

    def _log_message(self, message: str):
        if hasattr(self.ui, 'log_text_edit'):
            self.ui.log_text_edit.append(message)
        else:
            print(f"LOG: {message}")

    def _configure_actions(self):
        if hasattr(self.ui, 'actionExit'):
            self.ui.actionExit.setIcon(QIcon.fromTheme("application-exit"))
            self.ui.actionExit.setStatusTip("Exit application")
        if hasattr(self.ui, 'actionRecord'):
            self.ui.actionRecord.setStatusTip("Arm/Disarm data recording for the next Manual Run")

    def _configure_status_bar(self):
        if hasattr(self.ui, 'statusbar'):
            self.ui.statusbar.showMessage("Ready. Please select a port and connect.", 0)
            self.recording_status_indicator = QLabel("")
            self.recording_status_indicator.setObjectName("recording_status_indicator")
            self.recording_status_indicator.setStyleSheet("color: red; margin-left: 10px; font-weight: bold;")
            self.ui.statusbar.addPermanentWidget(self.recording_status_indicator)
            self.recording_status_indicator.hide()

    def _setup_validators(self):
        # Manual (cm)
        self.manual_start_cm_validator = QDoubleValidator(0.0, 1.0, 3, self)
        self.manual_start_cm_validator.setNotation(QDoubleValidator.StandardNotation)
        self.ui.start_pos_input.setValidator(self.manual_start_cm_validator)
        self.ui.start_pos_input.setPlaceholderText("cm (e.g. 1.234)")

        self.manual_end_cm_validator = QDoubleValidator(0.0, 1.0, 3, self)
        self.manual_end_cm_validator.setNotation(QDoubleValidator.StandardNotation)
        self.ui.end_pos_input.setValidator(self.manual_end_cm_validator)
        self.ui.end_pos_input.setPlaceholderText("cm (e.g. 10.500)")

        # DAC voltage
        self.dac_validator = QDoubleValidator(0.1, 4.0, 1, self)
        self.dac_validator.setNotation(QDoubleValidator.StandardNotation)
        self.ui.dac_voltage_input.setValidator(self.dac_validator)

    def _connect_signals(self):
        # Menu
        self.ui.actionExit.triggered.connect(self.close)

        # Connection controls
        self.ui.refresh_button.clicked.connect(self.serial_handler.list_ports)
        self.ui.connect_button.clicked.connect(self._attempt_connection)
        self.ui.disconnect_button.clicked.connect(self._disconnect_device)

        # Serial handler signals
        self.serial_handler.ports_updated.connect(self._update_port_combo)
        self.serial_handler.connected.connect(self._handle_connection_success)
        self.serial_handler.disconnected.connect(self._handle_disconnection_signal)
        self.serial_handler.error_occurred.connect(self._show_serial_error)

        # Route serial data to DataProcessor (worker thread)
        self.serial_handler.data_received.connect(self.data_processor.process_raw_data)
        # Route processed data to UI slot
        self.data_processor.data_ready_for_ui.connect(self._update_ui_with_processed_data)

        # Motor controls
        self.ui.move_left_button.pressed.connect(self._start_move_left)
        self.ui.move_left_button.released.connect(self._stop_move)
        self.ui.move_right_button.pressed.connect(self._start_move_right)
        self.ui.move_right_button.released.connect(self._stop_move)
        self.ui.calibrate_button.clicked.connect(self._calibrate_motor)
        self.ui.speed_button_group.buttonClicked.connect(self._update_motor_speed)

        # Pause button flash timer
        self.stop_flash_timer.timeout.connect(self._toggle_pause_button_flash)

        # DAC
        self.ui.dac_set_button.clicked.connect(self._set_dac_voltage)
        self.ui.dac_voltage_input.returnPressed.connect(self._set_dac_voltage)

        # MUX slider
        if hasattr(self.ui, 'res_slider'):
            self.ui.res_slider.valueChanged.connect(self._on_mux_changed)

        # Plot update timer
        self.plot_update_timer.timeout.connect(self.plot.update_plots)

    # ------------------ Serial/UI slots ------------------
    @Slot(list)
    def _update_port_combo(self, ports):
        current_selection = self.ui.port_combo.currentText()
        self.ui.port_combo.clear()
        self.ui.connection_status_label.setText("")
        self.ui.connection_status_label.setStyleSheet("")
        if ports:
            self.ui.port_combo.addItems(ports)
            if current_selection in ports:
                self.ui.port_combo.setCurrentText(current_selection)
            self.ui.port_combo.setEnabled(True)
            if not self.serial_handler.is_connected():
                self.ui.connect_button.setEnabled(True)
            self._log_message("Serial ports updated.")
        else:
            self.ui.port_combo.addItem("No ports found")
            self.ui.port_combo.setEnabled(False)
            self.ui.connect_button.setEnabled(False)
            self._log_message("No serial ports found.")

    @Slot()
    def _attempt_connection(self):
        selected_port = self.ui.port_combo.currentText()
        if selected_port and selected_port != "No ports found":
            self.ui.statusbar.showMessage(f"Connecting to {selected_port}...", 0)
            self.ui.connection_status_label.setText("Connecting...")
            self.ui.connection_status_label.setStyleSheet("color: blue; font-weight: bold; padding-left: 10px;")
            self._update_controls_state(connecting=True)
            self.serial_handler.connect(selected_port)
        else:
            self._show_serial_error("Please select a valid port.")
            self.ui.connection_status_label.setText("Select Port!")
            self.ui.connection_status_label.setStyleSheet("color: orange; font-weight: bold; padding-left: 10px;")

    @Slot(str)
    def _show_serial_error(self, error_message):
        if hasattr(self.ui, 'statusbar'):
            self.ui.statusbar.showMessage(f"Error: {error_message}", 5000)
        self._log_message(f"SERIAL_ERROR: {error_message}")
        if hasattr(self.ui, 'connection_status_label'):
            if ("Connection Error" in error_message or
                "Failed to open port" in error_message or
                "Serial Error" in error_message or
                "Cannot send command" in error_message):
                self.ui.connection_status_label.setText("Connection Failed")
                self.ui.connection_status_label.setStyleSheet("color: orange; font-weight: bold; padding-left: 10px;")
                self._set_conn_state_border("failed")
            elif "No port selected" in error_message:
                self.ui.connection_status_label.setText("Select Port!")
                self.ui.connection_status_label.setStyleSheet("color: orange; font-weight: bold; padding-left: 10px;")
                self._set_conn_state_border("failed")
        self._update_controls_state(connected=False, calibrated=False)

    @Slot()
    def _handle_connection_success(self):
        port_name = self.serial_handler.serial_port.name
        self.ui.statusbar.showMessage(f"Connected to {port_name}", 5000)
        self._log_message(f"Successfully connected to {port_name}.")
        self.ui.connection_status_label.setText("Connected")
        self.ui.connection_status_label.setStyleSheet("color: green; font-weight: bold; padding-left: 10px;")
        self._set_conn_state_border("connected")

        # Reset state
        self.is_calibrated = False
        self.microstep_factor = 1
        self.ui.radio_fast.setChecked(True)
        self.current_dac_voltage = 1.0
        self.ui.dac_current_label.setText(f"{self.current_dac_voltage:.1f} V")

        # Reset validators
        if hasattr(self, 'manual_start_cm_validator'):
            self.manual_start_cm_validator.setTop(1.0)
            self.manual_end_cm_validator.setTop(1.0)

        # Reset manual state (and clear plots)
        self.manual._reset_manual_run_state(completed=False)

        # Update controls
        self._update_controls_state(connected=True, calibrated=False)
        self._update_position_display()

    @Slot()
    def _disconnect_device(self):
        self._log_message("Disconnect button clicked by user.")
        self._send_stop_command()
        self.serial_handler.disconnect()
        
    def _set_conn_state_border(self, state: str):
        """
        state: "connected" | "failed" | "disconnected"
        این متد property را ست می‌کند و repolish انجام می‌دهد تا QSS اعمال شود.
        """
        if hasattr(self.ui, "connection_group"):
            self.ui.connection_group.setProperty("connState", state)
            self.ui.connection_group.style().unpolish(self.ui.connection_group)
            self.ui.connection_group.style().polish(self.ui.connection_group)
            self.ui.connection_group.update()

    @Slot()
    def _handle_disconnection_signal(self):
        self.ui.statusbar.showMessage("Disconnected. Select a port to connect.", 0)
        self._log_message("Disconnected from serial port (signal received).")
        self.ui.connection_status_label.setText("Disconnected")
        self.ui.connection_status_label.setStyleSheet("color: red; font-weight: bold; padding-left: 10px;")
        self._set_conn_state_border("disconnected")

        self.is_calibrated = False
        self.manual._reset_manual_run_state(completed=False)
        self._update_controls_state(connected=False, calibrated=False)
        self._update_position_display()

    @Slot(dict)
    def _update_ui_with_processed_data(self, data_dict):
        # تفویض بخش‌های Manual/Recording/Displacement به ManualController
        self.manual.handle_processed_data(data_dict)

        # Live plots
        if 'live_plot_point' in data_dict:
            voltage, resistance = data_dict['live_plot_point']
            self.plot.append_live(voltage, resistance, t=time.time())

        # Position
        if 'position_update' in data_dict:
            self.current_motor_position = data_dict['position_update']
            self._update_position_display()

        # Smart ADC logging
        if 'latest_adc_voltage' in data_dict:
            v = data_dict['latest_adc_voltage']
            if self.last_logged_adc_voltage is None or abs(v - self.last_logged_adc_voltage) >= self.adc_log_threshold:
                log_msg = f"Initial ADC Reading: {v:.4f} V" if self.last_logged_adc_voltage is None else f"ADC Value Changed: {v:.4f} V"
                self._log_message(log_msg)
                self.last_logged_adc_voltage = v

        # Status events
        if 'total_steps_update' in data_dict:
            self.total_motor_steps = data_dict['total_steps_update']
            self._log_message(f"Debug: Received total steps: {self.total_motor_steps}")
            if self.total_motor_steps > 0:
                max_cm_range = self.total_motor_steps * CM_PER_FULL_STEP
                self.manual_start_cm_validator.setTop(max_cm_range)
                self.manual_end_cm_validator.setTop(max_cm_range)
                self._log_message(f"Debug: Manual input range set to 0.000 - {max_cm_range:.3f} cm")
            self._update_position_display()

        if 'calibration_done' in data_dict:
            self.is_calibrated = True
            self._log_message("Arduino reported: Calibration Complete.")
            self._update_controls_state(connected=True, calibrated=True)
            self._update_position_display()
            self.ui.statusbar.showMessage("Calibration Complete. Ready to move.", 3000)

        # Sync GUI-selected MUX with device-confirmed channel to avoid R mismatch
        if 'mux_channel_confirmed' in data_dict:
            ch = int(data_dict['mux_channel_confirmed'])
            ch = max(0, min(15, ch))
            # prev/current bookkeeping
            self.prev_mux_channel = getattr(self, 'current_mux_channel', 0)
            self.prev_selected_res_kohm = getattr(self, 'current_selected_res_kohm', RES_VALUES[self.prev_mux_channel])
            self.current_mux_channel = ch
            self.current_selected_res_kohm = float(RES_VALUES[ch])
            if hasattr(self.ui, 'res_slider'):
                # move slider if out of sync
                if int(self.ui.res_slider.value()) != ch:
                    self.ui.res_slider.blockSignals(True)
                    self.ui.res_slider.setValue(ch)
                    self.ui.res_slider.blockSignals(False)
            if hasattr(self.ui, 'res_value_label'):
                self.ui.res_value_label.setText(f"{RES_VALUES[ch]} kΩ")
            self._log_message(f"MUX channel confirmed by device: {ch} (Rm={RES_VALUES[ch]} kΩ)")

        if 'calibration_started' in data_dict:
            self.ui.statusbar.showMessage("Calibration in progress...", 0)
            self._update_controls_state(calibrating=True)

    def _update_position_display(self, pos_microsteps=None, total_full_steps=None):
        start_cm_str = "0.000"
        current_cm_str = "---"
        total_cm_str = "---"

        current_pos_to_use = self.current_motor_position if pos_microsteps is None else pos_microsteps
        total_steps_to_use = self.total_motor_steps if total_full_steps is None else total_full_steps

        if self.serial_handler.is_connected():
            if self.is_calibrated:
                try:
                    if isinstance(total_steps_to_use, int) and total_steps_to_use >= 0:
                        total_cm = total_steps_to_use * CM_PER_FULL_STEP
                        total_cm_str = f"{total_cm:.3f}"
                    else:
                        total_cm_str = "Invalid"
                except Exception:
                    total_cm_str = "ErrorCalc"
            if self.is_calibrated:
                try:
                    if isinstance(current_pos_to_use, int) and self.microstep_factor > 0:
                        equivalent_full_steps = current_pos_to_use / self.microstep_factor
                        current_cm = equivalent_full_steps * CM_PER_FULL_STEP
                        current_cm_str = f"{current_cm:.3f}"
                    else:
                        current_cm_str = "FactorErr"
                except Exception:
                    current_cm_str = "CalcErr"

        self.ui.position_label.setText(f"Pos: {start_cm_str} cm | {current_cm_str} cm | {total_cm_str} cm")

    def _update_controls_state(self, connected=None, calibrated=None, connecting=False, calibrating=False):
        is_connected_now = self.serial_handler.is_connected() if connected is None else connected
        is_calibrated_now = self.is_calibrated if calibrated is None else calibrated
        is_motor_busy_critical = connecting or calibrating or self.is_manual_running

        # Connection
        can_connect = (not is_connected_now and not is_motor_busy_critical and
                       self.ui.port_combo.count() > 0 and self.ui.port_combo.currentText() != "No ports found")
        self.ui.port_combo.setEnabled(not is_connected_now and not is_motor_busy_critical)
        self.ui.refresh_button.setEnabled(not is_connected_now and not is_motor_busy_critical)
        self.ui.connect_button.setEnabled(can_connect)
        self.ui.disconnect_button.setEnabled(is_connected_now and not connecting and not calibrating and not self.is_manual_running)
        if hasattr(self.ui, 'res_slider'):
            self.ui.res_slider.setEnabled(is_connected_now)

        # Motor/calibration
        can_calibrate_now = (
            is_connected_now and
            not calibrating and
            not self.is_manual_running and
            not self.is_manual_paused
        )
        self.ui.calibrate_button.setEnabled(can_calibrate_now)

        can_set_speed_now = is_connected_now and (not is_motor_busy_critical or self.is_manual_paused)
        self.ui.radio_fast.setEnabled(can_set_speed_now)
        self.ui.radio_smooth.setEnabled(can_set_speed_now)
        self.ui.radio_slow.setEnabled(can_set_speed_now)

        can_jog_now = is_connected_now and is_calibrated_now and (not is_motor_busy_critical or self.is_manual_paused)
        self.ui.move_left_button.setEnabled(can_jog_now)
        self.ui.move_right_button.setEnabled(can_jog_now)

        can_initiate_manual_run = is_connected_now and is_calibrated_now and not is_motor_busy_critical
        self.ui.manual_run_button.setEnabled(can_initiate_manual_run or (is_connected_now and self.is_manual_paused))
        self.ui.manual_pause_button.setEnabled(is_connected_now and self.is_manual_running and not self.is_manual_paused)
        self.ui.manual_full_stop_button.setEnabled(is_connected_now and (self.is_manual_running or self.is_manual_paused))

        can_edit_manual_inputs = can_initiate_manual_run or self.is_manual_paused
        self.ui.start_pos_input.setEnabled(can_edit_manual_inputs)
        self.ui.end_pos_input.setEnabled(can_edit_manual_inputs)
        self.ui.cycle_spinbox.setEnabled(can_edit_manual_inputs)

        can_set_references = is_connected_now and not connecting
        self.ui.dac_voltage_input.setEnabled(can_set_references)
        self.ui.dac_set_button.setEnabled(can_set_references)

        if hasattr(self.ui, 'actionRecord'):
            self.ui.actionRecord.setEnabled(is_connected_now)

    @Slot()
    def _toggle_pause_button_flash(self):
        if not hasattr(self.ui, 'manual_pause_button'):
            return
        if self.is_manual_paused:
            if self.stop_button_flash_state:
                self.ui.manual_pause_button.setStyleSheet("color: orange; background-color: yellow; font-weight: bold;")
            else:
                self.ui.manual_pause_button.setStyleSheet("color: orange; background-color: none; font-weight: bold;")
            self.stop_button_flash_state = not self.stop_button_flash_state
        else:
            self.ui.manual_pause_button.setStyleSheet("color: orange;")
            if self.stop_flash_timer.isActive():
                self.stop_flash_timer.stop()

    def _get_current_speed_name(self) -> str:
        if self.ui.radio_fast.isChecked(): return "FAST"
        elif self.ui.radio_smooth.isChecked(): return "SMOOTH"
        elif self.ui.radio_slow.isChecked(): return "SLOW"
        return "FAST"

    def _send_stop_command(self):
        if self.serial_handler.is_connected():
            self.serial_handler.send_command("$STOP")
            self.serial_handler.send_command("$REC_OFF")
            self._log_message("Sent: $STOP (for normal continuous move)")

    @Slot()
    def _set_dac_voltage(self):
        if not self.serial_handler.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to the device first.")
            return
        try:
            voltage_text = self.ui.dac_voltage_input.text()
            state, _, _ = self.dac_validator.validate(voltage_text, 0)
            if state != QValidator.Acceptable:
                QMessageBox.warning(
                    self,
                    "Invalid Voltage",
                    f"Please enter a voltage between {self.dac_validator.bottom():.1f} "
                    f"and {self.dac_validator.top():.1f} V (one decimal place)."
                )
                return
            voltage_to_set = float(voltage_text)
            command = f"SET_DAC_V:{voltage_to_set:.1f}"
            self._log_message(f"Command: {command}")

            if self.serial_handler.send_command(command):
                self.current_dac_voltage = voltage_to_set
                self.ui.dac_current_label.setText(f"{self.current_dac_voltage:.1f} V")

                # Arm a short resistance mute window to hide DAC-change spike on plots
                try:
                    now = time.time()
                    dur = getattr(self.data_processor, 'rt_mute_window_s', 0.1)
                    self.data_processor.rt_mute_until = now + dur
                except Exception:
                    pass

                self.ui.statusbar.showMessage(f"voltage set to {self.current_dac_voltage:.1f} V.", 3000)
            else:
                QMessageBox.critical(self, "Send Error", "Failed to send DAC command to device.")
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Invalid voltage format. Please enter a number.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An unexpected error occurred while setting voltage: {e}")

    @Slot(int)
    def _on_mux_changed(self, value: int):
        ch = max(0, min(15, int(value)))
        # قبل از تغییر، قبلی را ذخیره کن
        self.prev_mux_channel = getattr(self, 'current_mux_channel', 0)
        self.prev_selected_res_kohm = getattr(self, 'current_selected_res_kohm', RES_VALUES[self.prev_mux_channel])

        self.current_mux_channel = ch
        self.current_selected_res_kohm = float(RES_VALUES[ch])
        if hasattr(self.ui, 'res_value_label'):
            self.ui.res_value_label.setText(f"{RES_VALUES[ch]} kΩ")
        if hasattr(self.ui, 'res_slider'):
            if int(self.ui.res_slider.value()) != ch:
                self.ui.res_slider.blockSignals(True)
                self.ui.res_slider.setValue(ch)
                self.ui.res_slider.blockSignals(False)

        # ارسال فرمان به دستگاه
        if self.serial_handler.is_connected():
            ok = self.serial_handler.send_command(f"SET_RES_MUX:{ch}")
            if not ok:
                self._log_message(f"Warning: Failed to send SET_RES_MUX:{ch}")
        try:
            now = time.time()
            dur = getattr(self.data_processor, 'rt_mute_window_s', 0.1)
            self.data_processor.rt_mute_until = now + dur
        except Exception:
            pass

        # Gate کوتاه (هماهنگ با MUX_MUTE_MS آردوینو ~50ms)
        self.mux_mute_until = time.time() + 0.1
        self._log_message(f"MUX change requested -> ch={ch}, Rm={RES_VALUES[ch]} kΩ; rt_mute ~{int(dur*1000)}ms, mux_mute 50ms")

    @Slot()
    def _start_move_left(self):
        if not self.serial_handler.is_connected() or not self.is_calibrated:
            self._log_message("Cannot move: Not connected or not calibrated.")
            return
        if self.is_manual_paused:
            target_us = self.current_manual_run_start_abs_steps * self.microstep_factor
            if self.current_motor_position > target_us:
                self.serial_handler.send_command(f"$START_LEFT_LIMITED:{target_us}")
            else:
                self._log_message("Already at/before Start (no jog needed).")
        elif not self.is_manual_running and not self._is_calibrating():
            self._log_message("Command: Start Normal Move Left (Pressed)")
            self.serial_handler.send_command("$START_LEFT")
        else:
            self._log_message("Cannot start normal move left: Manual run active or calibrating.")

    @Slot()
    def _start_move_right(self):
        if not self.serial_handler.is_connected() or not self.is_calibrated:
            self._log_message("Cannot move: Not connected or not calibrated.")
            return
        if self.is_manual_paused:
            target_us = self.current_manual_run_end_abs_steps * self.microstep_factor
            if self.current_motor_position < target_us:
                self.serial_handler.send_command(f"$START_RIGHT_LIMITED:{target_us}")
            else:
                self._log_message("Already at/after End (no jog needed).")
        elif not self.is_manual_running and not self._is_calibrating():
            self._log_message("Command: Start Normal Move Right (Pressed)")
            self.serial_handler.send_command("$START_RIGHT")
        else:
            self._log_message("Cannot start normal move right: Manual run active or calibrating.")

    @Slot()
    def _stop_move(self):
        if not self.is_manual_running and not self.is_manual_paused and not self._is_calibrating():
            self._log_message("Command: Stop Normal Move (Released or Internal)")
            self._send_stop_command()

    @Slot(object)
    def _update_motor_speed(self, button):
        if not self.serial_handler.is_connected():
            self._log_message("Cannot change speed: Not connected.")
            if self.microstep_factor == 1: self.ui.radio_fast.setChecked(True)
            elif self.microstep_factor == 2: self.ui.radio_smooth.setChecked(True)
            else: self.ui.radio_slow.setChecked(True)
            return

        button_id = self.ui.speed_button_group.id(button)
        if button_id == 1: speed_command, speed_name, new_factor = "$SPEED_FAST", "Fast", 1
        elif button_id == 2: speed_command, speed_name, new_factor = "$SPEED_SMOOTH", "Smooth", 2
        elif button_id == 3: speed_command, speed_name, new_factor = "$SPEED_SLOW", "Slow", 4
        else:
            self._log_message(f"Warning: Unknown speed button ID: {button_id}")
            return

        if self.microstep_factor != new_factor:
            self._log_message(f"Command: Set Speed to {speed_name} (Factor: {new_factor})")
            self._send_stop_command()
            if self.is_manual_running or self.is_manual_paused:
                if self.serial_handler.send_command("$MANUAL_STOP"):
                    self._log_message("Manual run paused before speed change.")
                else:
                    self._log_message("Warning: Could not send MANUAL_STOP before speed change.")

            if self.serial_handler.send_command(speed_command):
                self.microstep_factor = new_factor
                self._update_position_display()
                self.ui.statusbar.showMessage(f"Speed set to {speed_name}.", 3000)
            else:
                QMessageBox.critical(self, "Send Error", f"Failed to send {speed_name} command.")
                if self.microstep_factor == 1: self.ui.radio_fast.setChecked(True)
                elif self.microstep_factor == 2: self.ui.radio_smooth.setChecked(True)
                else: self.ui.radio_slow.setChecked(True)
        else:
            self._log_message(f"Speed already set to {speed_name}.")

    @Slot()
    def _calibrate_motor(self):
        """
        Sends the calibration command to Arduino.
        Only when connected, not calibrating, and no manual run is active/paused.
        """
        if self.serial_handler.is_connected() and \
        not self._is_calibrating() and \
        not self.is_manual_running and \
        not self.is_manual_paused:
            self._log_message("Command: Calibrate Motor")
            self._send_stop_command()  # توقف هر حرکت عادی قبلی
            if self.serial_handler.send_command("$CALIBRATE"):
                # UI در حالت کالیبراسیون
                self._update_controls_state(calibrating=True)
                if hasattr(self.ui, 'statusbar'):
                    self.ui.statusbar.showMessage("Calibration started...", 0)
            else:
                QMessageBox.critical(self, "Send Error", "Failed to send Calibrate command.")
        else:
            self._log_message("Cannot calibrate: Not connected or operation in progress.")

    def _is_calibrating(self) -> bool:
        if hasattr(self.ui, 'calibrate_button'):
            return (
                self.serial_handler.is_connected() and
                not self.ui.calibrate_button.isEnabled() and
                not self.is_manual_running and
                not self.is_manual_paused
            )
        return False

    def closeEvent(self, event):
        self._log_message("Closing application...")

        # Delegate final recording flushes to ManualController
        self.manual.on_close()

        # Stop motor and disconnect
        if self.serial_handler.is_connected():
            if self.is_manual_running or self.is_manual_paused:
                self._log_message("Sending Manual Abort on close.")
                self.serial_handler.send_command("$MANUAL_ABORT")
            self._send_stop_command()

        self.serial_handler.disconnect()

        # Stop timers
        if hasattr(self, 'plot_update_timer') and self.plot_update_timer.isActive():
            self.plot_update_timer.stop()
            self._log_message("Plot update timer stopped.")
        if hasattr(self, 'stop_flash_timer') and self.stop_flash_timer.isActive():
            self.stop_flash_timer.stop()
            self._log_message("Pause button flash timer stopped.")

        # Stop DataProcessor thread
        try:
            if self.data_processor_thread.isRunning():
                self.data_processor_thread.quit()
                if not self.data_processor_thread.wait(1000):
                    self.data_processor_thread.terminate()
            self._log_message("Data processor thread stopped.")
        except Exception:
            pass

        self._log_message("Cleanup complete. Exiting.")
        event.accept()

    @Slot()
    def _on_mux_slider_released(self):
        """ارسال SET_RES_MUX به آردوینو وقتی اسلایدر رها می‌شود."""
        value = int(self.ui.res_slider.value())
        self._on_mux_changed(value)

    @Slot(int)
    def _on_mux_slider_moved(self, value):
        """فقط بروزرسانی لیبل و مقدار انتخابی در هنگام حرکت اسلایدر (بدون ارسال سریال)."""
        value = max(0, min(15, int(value)))
        self.current_mux_channel = value
        self.current_selected_res_kohm = float(RES_VALUES[value])
        if hasattr(self.ui, 'res_value_label'):
            self.ui.res_value_label.setText(f"{RES_VALUES[value]} kΩ")


# --- Application Entry Point ---
if __name__ == "__main__":
    pg.setConfigOptions(antialias=True)
    pg.setConfigOption('background', 'w')

    app = QApplication(sys.argv)

    stylesheet_file = "style.qss"
    try:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        style_path = os.path.join(current_dir, stylesheet_file)
        if os.path.exists(style_path):
            with open(style_path, "r", encoding="utf-8") as f:
                app.setStyleSheet(f.read())
            print(f"Stylesheet '{stylesheet_file}' loaded successfully.")
        else:
            print(f"Warning: Stylesheet file '{stylesheet_file}' not found. Using default Qt style.")
    except Exception as e:
        print(f"Error loading stylesheet '{stylesheet_file}': {e}")

    window = MainWindow()
    window.show()
    sys.exit(app.exec())