# filename: manual_control.py
# Manual Control & Recording logic for Linear Rail Controller

import os
import csv
import time
from PySide6.QtCore import QObject, QTimer, Slot
from PySide6.QtWidgets import QFileDialog, QMessageBox
from PySide6.QtGui import QValidator

# ثابت‌های تبدیل – هم‌راستای main_app.py
CM_PER_FULL_STEP = 0.001  # 1 full-step = 0.001 cm
MM_PER_FULL_STEP = 0.01   # 1 full-step = 0.01 mm
MIN_DIFF_CM = 0.01        # حداقل فاصله مجاز Start/End = 0.1mm = 0.01cm


class ManualController(QObject):
    """
    - Run / Resume / Pause / Full Stop (Abort)
    - مدیریت ضبط فایل CSV (Arm/Start/Pause/Resume/Stop)
    - همگام‌سازی با PlotHandler و StatusBar Indicator
    - ادامه شماره سایکل‌ها بعد از Stop→No با csv_cycle_offset
    """

    def __init__(self, main_window, serial_handler, plot_handler, data_processor):
        super().__init__(parent=main_window)
        self.mw = main_window
        self.ui = main_window.ui
        self.serial = serial_handler
        self.plot = plot_handler
        self.dp = data_processor

        # تایمر نوشتن دوره‌ای روی فایل (هر 2.5s)
        self.file_write_timer = QTimer(self)
        self.file_write_interval = 2500
        self.file_write_timer.setInterval(self.file_write_interval)
        self.file_write_timer.timeout.connect(self._write_buffer_to_file)
        self.file_write_timer.start()

        # Manual state (روی MainWindow نگه داشته می‌شود)
        for name, val in [
            ('is_manual_running', False),
            ('is_manual_paused', False),
            ('manual_completed_cycles', 0),
            ('manual_run_original_start_cm', 0.0),
            ('manual_run_original_end_cm', 0.0),
            ('manual_run_original_cycles', 1),
            ('current_manual_run_start_abs_steps', 0),
            ('current_manual_run_end_abs_steps', 0),
            ('current_leg_start_abs_microsteps', 0),
            ('current_leg_is_going_to_end', True),
            ('current_manual_leg_max_dist_mm', 0.0),
            ('is_recording_armed', False),
            ('is_actively_recording', False),
            ('recording_filepath', ""),
            ('recorded_data_buffer', []),
            ('manual_run_start_time', 0),
            ('paused_relative_time', 0),
        ]:
            if not hasattr(self.mw, name):
                setattr(self.mw, name, val)

        # Offset شماره‌ی سایکل برای ادامه بعد از Stop→No
        self.csv_cycle_offset = 0

    # ---------------- Wiring UI ----------------
    def connect_ui_signals(self):
        if hasattr(self.ui, 'actionRecord'):
            self.ui.actionRecord.triggered.connect(self._handle_record_data_action)

        self.ui.manual_run_button.clicked.connect(self._handle_manual_run_resume)
        self.ui.manual_pause_button.clicked.connect(self._handle_manual_pause_toggle)
        self.ui.manual_full_stop_button.clicked.connect(self._handle_manual_full_stop)

        self.ui.start_pos_input.returnPressed.connect(self.ui.manual_run_button.click)
        self.ui.end_pos_input.returnPressed.connect(self.ui.manual_run_button.click)
        if hasattr(self.ui.cycle_spinbox, 'lineEdit') and self.ui.cycle_spinbox.lineEdit():
            self.ui.cycle_spinbox.lineEdit().returnPressed.connect(self.ui.manual_run_button.click)

    # ---------------- Data from DataProcessor ----------------
    def handle_processed_data(self, data_dict: dict):
        # نقطه‌های جابجایی
        if 'displacement_plot_point' in data_dict:
            distance, voltage, resistance = data_dict['displacement_plot_point']
            self.plot.append_displacement(distance, voltage, resistance)

        # ردیف CSV
        if 'record_row' in data_dict:
            self.mw.recorded_data_buffer.append(data_dict['record_row'])

        # سایکل تمام شد (از دستگاه) → مجموع = offset + n
        if 'cycle_done' in data_dict:
            n = int(data_dict['cycle_done'])
            total = self.csv_cycle_offset + n
            self.mw.manual_completed_cycles = total
            self.ui.manual_cycle_label.setText(f"Cycles Done: {self.mw.manual_completed_cycles}")
            self.mw._log_message(f"Manual cycle {self.mw.manual_completed_cycles} completed.")
            # لگ جدید با رنگ مطابق شماره کل
            self.plot.start_new_leg(cycle_index=self.mw.manual_completed_cycles)
            # آغاز لگ بعدی (Start -> End)
            self.mw.current_leg_start_abs_microsteps = self.mw.current_manual_run_start_abs_steps * self.mw.microstep_factor
            self.mw.current_leg_is_going_to_end = True

        # رسیدن به هدف (انتها/ابتدا)
        if 'manual_target_reached' in data_dict:
            self.mw._log_message("Arduino reported: Manual target reached.")
            if self.mw.current_leg_is_going_to_end:
                self.mw.current_leg_start_abs_microsteps = self.mw.current_manual_run_end_abs_steps * self.mw.microstep_factor
                self.mw.current_leg_is_going_to_end = False
            else:
                self.mw.current_leg_start_abs_microsteps = self.mw.current_manual_run_start_abs_steps * self.mw.microstep_factor
                self.mw.current_leg_is_going_to_end = True
            self.plot.start_new_leg()  # رنگ لگ دوم همان قبلی

        # Pause
        if 'manual_run_paused' in data_dict:
            if not self.mw.is_manual_paused:
                self._set_manual_paused_state()
            else:
                # اگر پیام تکراری بود، کنترل‌ها فعال بمانند
                self.ui.move_left_button.setEnabled(True)
                self.ui.move_right_button.setEnabled(True)
                self.ui.radio_fast.setEnabled(True)
                self.ui.radio_smooth.setEnabled(True)
                self.ui.radio_slow.setEnabled(True)
                self.ui.manual_run_button.setEnabled(True)

        # اتمام همه سایکل‌ها
        if 'all_manual_cycles_completed' in data_dict:
            if not self.mw.is_manual_paused and not self.mw.is_manual_running:
                # اتمام کار → ذخیره و بستن فایل و ریست رکورد/offset
                self._reset_manual_run_state(completed=True)

    def on_close(self):
        """هنگام خروج: نوشتن باقی‌مانده داده و خاموش کردن REC."""
        if self.mw.is_actively_recording and self.mw.recorded_data_buffer:
            self.serial.send_command("$REC_OFF")
            self.mw._log_message("Application closing. Writing final data batch...")
            data_to_write = self.mw.recorded_data_buffer[:]
            self.mw.recorded_data_buffer.clear()
            try:
                file_exists = os.path.exists(self.mw.recording_filepath)
                write_header = not file_exists or (file_exists and os.path.getsize(self.mw.recording_filepath) == 0)
                with open(self.mw.recording_filepath, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    if write_header:
                        meta_speed = self._safe_speed()
                        writer.writerow(['#META', 'Vi (V)', f'{self.mw.current_dac_voltage:.3f}', 'Rm (kOhm)', f'{float(self.mw.current_selected_res_kohm):.3f}', 'Speed', meta_speed, 'Microstep', str(self.mw.microstep_factor)])
                        writer.writerow(['Cycle', 'Time (s)', 'Relative Distance (mm)', 'ADC Voltage (V)', 'Resistance (kOhm)'])
                    writer.writerows(data_to_write)
                self.mw._log_message(f"Final {len(data_to_write)} points saved to {self.mw.recording_filepath}.")
            except Exception as e:
                self.mw._log_message(f"CRITICAL: Could not save remaining data on exit! Error: {e}")

    # ---------------- Internal helpers ----------------
    def _safe_speed(self) -> str:
        try:
            return self.mw._get_current_speed_name()
        except Exception:
            return "UNKNOWN"

    def _set_manual_paused_state(self):
        # فلگ‌ها
        self.mw.is_manual_paused = True
        self.mw.is_manual_running = False

        # متن دکمه‌ها
        self.ui.manual_run_button.setText("Resume")
        if hasattr(self.ui, 'manual_pause_button'):
            self.ui.manual_pause_button.setText("Resume")

        # فلاش Pause
        self.mw.stop_button_flash_state = False
        self.mw._toggle_pause_button_flash()
        self.mw.stop_flash_timer.start()

        # کنترل‌ها در Pause فعال
        self.ui.move_left_button.setEnabled(True)
        self.ui.move_right_button.setEnabled(True)
        self.ui.radio_fast.setEnabled(True)
        self.ui.radio_smooth.setEnabled(True)
        self.ui.radio_slow.setEnabled(True)
        self.ui.manual_run_button.setEnabled(True)

        self.mw._update_controls_state()

        # وضعیت رکورد → Pause
        if self.mw.is_actively_recording and hasattr(self.mw, 'recording_status_indicator'):
            self.mw.recording_status_indicator.setText("⏸️ Recording Paused...")
            self.mw.recording_status_indicator.setStyleSheet("color: orange; margin-left: 10px; font-weight: bold;")
            self.mw._log_message("Recording PAUSED due to manual run pause.")

        # رنگ دکمه Run در Pause می‌تواند پیش‌فرض باشد
        self.ui.manual_run_button.setStyleSheet("")

    def _reset_manual_run_state(self, completed=False):
        was_actively_recording_before_reset = self.mw.is_actively_recording

        self.mw.is_manual_running = False
        self.mw.is_manual_paused = False
        self.ui.manual_run_button.setText("Run")
        if hasattr(self.ui, 'manual_pause_button'):
            self.ui.manual_pause_button.setText("Pause")
            self.ui.manual_pause_button.setStyleSheet("color: orange;")
        if hasattr(self.mw, 'stop_flash_timer'):
            self.mw.stop_flash_timer.stop()

        # Finalize recording
        if was_actively_recording_before_reset:
            self.mw._log_message("Manual run ended/reset. Finalizing recording...")
            if self.mw.recorded_data_buffer:
                if self._write_recorded_data_to_file(append_always=True):
                    self.mw.recorded_data_buffer.clear()
                    self.mw._log_message(f"Remaining data for {self.mw.recording_filepath} saved.")
                else:
                    self.mw._log_message(f"Warning: Failed to save remaining data for {self.mw.recording_filepath}.")
            # REC خاموش + منو به حالت اولیه
            self.mw.is_actively_recording = False
            self.mw.is_recording_armed = False
            if hasattr(self.ui, 'actionRecord'):
                self.ui.actionRecord.setChecked(False)
                self.ui.actionRecord.setText("Record Data")
            if hasattr(self.mw, 'recording_status_indicator'):
                self.mw.recording_status_indicator.hide()
            if hasattr(self.ui, 'statusbar'):
                self.ui.statusbar.clearMessage()
            self.mw.recording_filepath = ""

        if completed:
            # پایان همه سایکل‌ها → offset ریست
            self.csv_cycle_offset = 0
            self.mw._log_message("Manual run finished all cycles.")
            self.ui.statusbar.showMessage("Manual run completed successfully.", 3000)
        else:
            self.mw._log_message("Manual run reset/aborted.")

        self.mw.manual_completed_cycles = 0
        self.ui.manual_cycle_label.setText("Cycles Done: 0")
        self.ui.manual_run_button.setStyleSheet("")
        # پاک کردن پلات‌های جابجایی
        self.plot.clear_all()

        self.mw._update_controls_state()

    def _write_buffer_to_file(self):
        if self.mw.is_actively_recording and self.mw.recorded_data_buffer:
            data_to_write = self.mw.recorded_data_buffer[:]
            self.mw.recorded_data_buffer.clear()

            if not self.mw.recording_filepath:
                self.mw._log_message("Error: Recording is active but file path is missing.")
                self.mw.recorded_data_buffer = data_to_write + self.mw.recorded_data_buffer
                return

            try:
                file_exists = os.path.exists(self.mw.recording_filepath)
                write_header = not file_exists or (file_exists and os.path.getsize(self.mw.recording_filepath) == 0)
                with open(self.mw.recording_filepath, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    if write_header:
                        meta_speed = self._safe_speed()
                        writer.writerow(['#META', 'Vi (V)', f'{self.mw.current_dac_voltage:.3f}', 'Rm (kOhm)', f'{float(self.mw.current_selected_res_kohm):.3f}', 'Speed', meta_speed, 'Microstep', str(self.mw.microstep_factor)])
                        writer.writerow(['Cycle', 'Time (s)', 'Relative Distance (mm)', 'ADC Voltage (V)', 'Resistance (kOhm)'])
                    writer.writerows(data_to_write)
                self.mw._log_message(f"Saved a batch of {len(data_to_write)} data points.")
            except Exception as e:
                self.mw._log_message(f"Error during periodic file write: {e}")
                self.mw.recorded_data_buffer = data_to_write + self.mw.recorded_data_buffer

    def _write_recorded_data_to_file(self, append_always=True):
        if not self.mw.recording_filepath:
            self.mw._log_message("Error: No recording file path specified for writing.")
            return False
        if not self.mw.recorded_data_buffer:
            self.mw._log_message("Info: No data in buffer to write.")
            return True
        self._write_buffer_to_file()
        return True

    # ---------------- UI Actions ----------------
    @Slot(bool)
    def _handle_record_data_action(self, checked):
        if checked:
            default_filename = f"rail_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            filePath, _ = QFileDialog.getSaveFileName(
                self.mw,
                "Choose File to Save Recorded Data",
                default_filename,
                "CSV Files (*.csv);;All Files (*)"
            )
            if filePath:
                self.mw.recording_filepath = filePath
                self.mw.is_recording_armed = True
                self.ui.actionRecord.setText("Cancel Recording Setup")
                if hasattr(self.mw, 'recording_status_indicator'):
                    self.mw.recording_status_indicator.setText("⚪ Armed: Ready to record")
                    self.mw.recording_status_indicator.setStyleSheet("color: gray; margin-left: 10px; font-weight: bold;")
                    self.mw.recording_status_indicator.show()
                self.mw._log_message(f"Recording ARMED. Data will be saved to: {self.mw.recording_filepath}")
            else:
                self.ui.actionRecord.setChecked(False)
                self.mw._log_message("Recording setup cancelled by user (no file selected).")
        else:
            # کاربر منو را لغو کرد
            if self.mw.is_actively_recording:
                self.mw._log_message("Recording STOPPED by menu action.")
                self._write_buffer_to_file()
            elif self.mw.is_recording_armed:
                self.mw._log_message("Recording setup CANCELLED by menu action.")
            self.mw.is_recording_armed = False
            self.mw.is_actively_recording = False
            self.mw.recording_filepath = ""
            self.mw.recorded_data_buffer = []
            self.ui.actionRecord.setText("Record Data")
            if hasattr(self.mw, 'recording_status_indicator'):
                self.mw.recording_status_indicator.hide()
            if hasattr(self.ui, 'statusbar'):
                self.ui.statusbar.clearMessage()

    @Slot()
    def _handle_manual_full_stop(self):
        # Abort Manual
        if self.mw.is_manual_running or self.mw.is_manual_paused:
            self.mw._log_message("Manual FULL STOP (Abort).")
            # REC موقتاً خاموش برای دستگاه (OLED → دایره توخالی)
            self.serial.send_command("$REC_OFF")

            if self.serial.send_command("$MANUAL_ABORT"):
                # پرسش ادامه با مقادیر قبلی یا شروع جدید
                reply = QMessageBox.question(
                    self.mw, "New Values?",
                    "Do you want to start a NEW run with new values?\nYes = enter new values\nNo = continue with previous values",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No
                )

                if reply == QMessageBox.No:
                    # ادامه با مقادیر قبلی → شماره سایکل‌ها از ادامه ادامه دهد
                    # Offset = تعداد سایکل‌های انجام‌شده تاکنون
                    self.csv_cycle_offset = int(self.mw.manual_completed_cycles)

                    # اگر قبلاً Recording فعال بود، ادامه می‌دهیم
                    if self.mw.is_actively_recording and hasattr(self.mw, 'recording_status_indicator'):
                        self.mw.recording_status_indicator.setText("🔴 Recording...")
                        self.mw.recording_status_indicator.setStyleSheet("color: red; margin-left: 10px; font-weight: bold;")
                        self.mw.recording_status_indicator.show()
                        self.serial.send_command("$REC_ON")  # OLED → دایره توپر

                    # ارسال دوباره MANUAL_RUN با مقادیر قبلی
                    if self.mw.total_motor_steps > 0 and self.serial.is_connected():
                        start_steps = self.mw.current_manual_run_start_abs_steps
                        end_steps   = self.mw.current_manual_run_end_abs_steps
                        cycles      = self.mw.manual_run_original_cycles
                        ok = self.serial.send_command(f"$MANUAL_RUN:{start_steps}:{end_steps}:{cycles}")
                        if ok:
                            # وضعیتِ Run فعال
                            self.mw.is_manual_running = True
                            self.mw.is_manual_paused = False
                            # دکمه Run سبز شود
                            self.ui.manual_run_button.setStyleSheet("background-color:#28a745; color:white; font-weight:bold;")
                            self.ui.manual_run_button.setText("Run")
                            self.ui.manual_pause_button.setText("Pause")
                            self.ui.manual_pause_button.setStyleSheet("color: orange;")
                            self.mw._update_controls_state()
                            self.mw._log_message("Continuing previous manual run with cached values.")
                        else:
                            self.mw._log_message("Failed to continue previous manual run.")
                    else:
                        self.mw._log_message("No cached manual values to continue.")
                else:
                    # Yes → شروع جدید: ضبط بسته و UI/منو ریست و offset=0
                    self._reset_manual_run_state(completed=False)
                    self.csv_cycle_offset = 0
                    # ورودی‌ها برای شروع جدید
                    self.ui.start_pos_input.clear()
                    self.ui.end_pos_input.clear()
                    self.ui.cycle_spinbox.setValue(1)
                    self.mw._update_controls_state()
                    self.ui.statusbar.showMessage("Manual aborted. Enter new values.", 3000)
            else:
                QMessageBox.warning(self.mw, "Send Error", "Could not send manual abort command to Arduino.")
        else:
            self.mw._log_message("STOP pressed, but no manual run was active/paused.")

    @Slot()
    def _handle_manual_pause_toggle(self):
        # Pause
        if self.mw.is_manual_running and not self.mw.is_manual_paused:
            self.mw._log_message("Command: Manual Pause button clicked.")
            if self.serial.send_command("$MANUAL_STOP"):
                self.mw._log_message("Manual Pause command sent to device microcontroller.")
                # زمان برای ادامه‌ی صحیح
                if self.mw.is_actively_recording:
                    try:
                        self.mw.paused_relative_time = time.time() - self.mw.manual_run_start_time
                    except Exception:
                        self.mw.paused_relative_time = 0
                    if hasattr(self.mw, 'recording_status_indicator'):
                        self.mw.recording_status_indicator.setText("⏸️ Recording Paused...")
                        self.mw.recording_status_indicator.setStyleSheet("color: orange; margin-left: 10px; font-weight: bold;")
                        self.serial.send_command("$REC_PAUSE")
                    self.mw._log_message("Recording PAUSED (pending Arduino confirmation).")

                # Fallback
                QTimer.singleShot(250, lambda: (not self.mw.is_manual_paused) and self._set_manual_paused_state())
            else:
                try:
                    QMessageBox.critical(self.mw, "Send Error", "Failed to send Manual Pause command.")
                except Exception:
                    pass
            return

        # Resume
        if self.mw.is_manual_paused:
            self.mw._log_message("Command: Manual Resume button (from Pause toggle) clicked.")
            self._handle_manual_run_resume()
            return

        self.mw._log_message("Pause button clicked but not in a pausable/resumable state.")

    @Slot()
    def _handle_manual_run_resume(self):
        """
        Run یا Resume:
        - اگر paused بود: RESUME (یا NEW اگر پارامترها عوض شده)
        - اگر paused نبود: NEW manual run با اعتبارسنجی
        """
        if not self.serial.is_connected():
            QMessageBox.warning(self.mw, "Not Connected", "Please connect to the device first.")
            return
        if not self.mw.is_calibrated:
            QMessageBox.warning(self.mw, "Not Calibrated", "Motor must be calibrated first.")
            return

        # RESUME
        if self.mw.is_manual_paused:
            self.mw._log_message("Attempting to RESUME manual run...")
            inputs_changed = False
            current_start_cm_text = self.ui.start_pos_input.text()
            current_end_cm_text = self.ui.end_pos_input.text()
            current_cycles_val = self.ui.cycle_spinbox.value()
            try:
                if float(current_start_cm_text) != self.mw.manual_run_original_start_cm or \
                   float(current_end_cm_text) != self.mw.manual_run_original_end_cm or \
                   current_cycles_val != self.mw.manual_run_original_cycles:
                    inputs_changed = True
            except ValueError:
                inputs_changed = True

            if inputs_changed:
                reply = QMessageBox.question(
                    self.mw, "Parameters Changed",
                    "Manual run parameters have been modified while paused.\n"
                    "Do you want to start a NEW run with these new values?\n\n"
                    "(Choosing 'No' will resume the previous run with its original parameters.)",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )
                if reply == QMessageBox.Yes:
                    self.mw._log_message("User chose to start a NEW manual run after changing parameters during pause.")
                    self._reset_manual_run_state(completed=False)
                    self.mw.is_manual_paused = False
                    self._handle_manual_run_resume()
                    return
                else:
                    self.mw._log_message("User chose to RESUME with original parameters.")
                    self.ui.start_pos_input.setText(f"{self.mw.manual_run_original_start_cm:.3f}")
                    self.ui.end_pos_input.setText(f"{self.mw.manual_run_original_end_cm:.3f}")
                    self.ui.cycle_spinbox.setValue(self.mw.manual_run_original_cycles)

            # ارسال سرعت فعلی
            current_speed_cmd = f"$SPEED_{self.mw._get_current_speed_name()}"
            self.mw._log_message(f"Sending current speed setting ({current_speed_cmd}) before resume.")
            if not self.serial.send_command(current_speed_cmd):
                QMessageBox.warning(self.mw, "Send Error", "Could not send speed command before resuming.")
                return

            # Resume
            if self.serial.send_command("$MANUAL_RESUME"):
                self.mw.is_manual_paused = False
                self.mw.is_manual_running = True
                self.ui.manual_run_button.setText("Run")
                self.ui.manual_pause_button.setText("Pause")
                self.ui.manual_pause_button.setStyleSheet("color: orange;")
                self.mw.stop_flash_timer.stop()
                # Run دکمه سبز
                self.ui.manual_run_button.setStyleSheet("background-color:#28a745; color:white; font-weight:bold;")
                # Resume Recording (اگر armed بود)
                if self.mw.is_recording_armed and not self.mw.is_actively_recording:
                    self.mw.is_actively_recording = True
                    paused_rel = getattr(self.mw, 'paused_relative_time', 0)
                    self.mw.manual_run_start_time = time.time() - paused_rel
                    if hasattr(self.mw, 'paused_relative_time'):
                        del self.mw.paused_relative_time
                    self.mw._log_message("Recording RESUMED.")
                if self.mw.is_actively_recording and hasattr(self.mw, 'recording_status_indicator'):
                    self.mw.recording_status_indicator.setText("🔴 Recording...")
                    self.mw.recording_status_indicator.setStyleSheet("color: red; margin-left: 10px; font-weight: bold;")
                    self.mw.recording_status_indicator.show()
                    self.serial.send_command("$REC_ON")

                self.mw._update_controls_state()
                self.mw._log_message("Manual run RESUMED.")
            else:
                QMessageBox.critical(self.mw, "Send Error", "Failed to send Manual Resume command.")
            return

        # NEW RUN
        self.mw._log_message("Attempting to START new manual run...")
        try:
            start_cm_text = self.ui.start_pos_input.text()
            end_cm_text = self.ui.end_pos_input.text()
            start_cm = float(start_cm_text)
            end_cm = float(end_cm_text)
            cycles = self.ui.cycle_spinbox.value()

            # اعتبارسنجی
            start_state = self.mw.manual_start_cm_validator.validate(start_cm_text, 0)[0]
            end_state = self.mw.manual_end_cm_validator.validate(end_cm_text, 0)[0]
            max_allowable_cm = self.mw.total_motor_steps * CM_PER_FULL_STEP
            if self.mw.total_motor_steps == 0 and self.mw.is_calibrated:
                max_allowable_cm = 0.01
            elif not self.mw.is_calibrated:
                QMessageBox.warning(self.mw, "Not Calibrated", "Please calibrate the motor first to define travel range.")
                return

            if start_state != QValidator.Acceptable or end_state != QValidator.Acceptable:
                QMessageBox.warning(self.mw, "Invalid Input",
                                    f"Start/End positions must be valid numbers.\n"
                                    f"Allowed range: 0.000 to {max_allowable_cm:.3f} cm.")
                return

            if abs(start_cm - end_cm) < MIN_DIFF_CM:
                QMessageBox.warning(self.mw, "Invalid Input",
                                    "Start and End positions cannot be the same or too close (min diff 0.1mm).")
                return

            if not (0.0 <= start_cm <= max_allowable_cm and 0.0 <= end_cm <= max_allowable_cm):
                QMessageBox.warning(self.mw, "Out of Range",
                                    f"Start/End positions must be within the calibrated range "
                                    f"(0.000 to {max_allowable_cm:.3f} cm).")
                return

            # ذخیره پارامترهای اصلی
            self.mw.manual_run_original_start_cm = start_cm
            self.mw.manual_run_original_end_cm = end_cm
            self.mw.manual_run_original_cycles = cycles

            # تبدیل به فول‌استپ
            start_steps = round(start_cm / CM_PER_FULL_STEP)
            end_steps = round(end_cm / CM_PER_FULL_STEP)
            start_steps = max(0, min(self.mw.total_motor_steps, start_steps))
            end_steps = max(0, min(self.mw.total_motor_steps, end_steps))
            if start_steps == end_steps:
                QMessageBox.warning(self.mw, "Invalid Input",
                                    "Converted Start and End steps are the same after rounding. "
                                    "Adjust cm values for a larger difference.")
                return

            # وضعیت داخلی Manual
            self.mw.current_manual_run_start_abs_steps = start_steps
            self.mw.current_manual_run_end_abs_steps = end_steps
            self.mw.current_leg_start_abs_microsteps = start_steps * self.mw.microstep_factor
            self.mw.current_leg_is_going_to_end = (end_steps > start_steps)
            self.mw.current_manual_leg_max_dist_mm = abs(end_cm - start_cm) * 10.0

            # offset سایکل برای Run جدید صفر شود
            self.csv_cycle_offset = 0
            self.mw.manual_completed_cycles = 0
            self.ui.manual_cycle_label.setText("Cycles Done: 0")

            # پلات‌ها
            self.plot.clear_all()
            self.plot.set_leg_max_range(self.mw.current_manual_leg_max_dist_mm)
            self.plot.start_new_leg(cycle_index=0)
            self.plot.update_plots()

            # ارسال دستور
            command = f"$MANUAL_RUN:{start_steps}:{end_steps}:{cycles}"
            self.mw._log_message(f"Command: {command} (Input cm: Start={start_cm:.3f}, End={end_cm:.3f})")
            self.mw._send_stop_command()

            if self.serial.send_command(command):
                self.mw.is_manual_running = True
                self.mw.is_manual_paused = False
                # Run سبز
                self.ui.manual_run_button.setStyleSheet("background-color:#28a745; color:white; font-weight:bold;")
                self.ui.manual_run_button.setText("Run")
                self.ui.manual_pause_button.setText("Pause")
                self.ui.manual_pause_button.setStyleSheet("color: orange;")

                # Recording اگر Armed است
                if self.mw.is_recording_armed and self.mw.recording_filepath:
                    self.mw.is_actively_recording = True
                    self.mw.manual_run_start_time = time.time()
                    self.mw.recorded_data_buffer = []
                    try:
                        is_new_file = (not os.path.exists(self.mw.recording_filepath) or
                                       os.path.getsize(self.mw.recording_filepath) == 0)
                        if is_new_file:
                            with open(self.mw.recording_filepath, 'w', newline='') as csvfile:
                                writer = csv.writer(csvfile)
                                meta_speed = self._safe_speed()
                                writer.writerow(['#META', 'Vi (V)', f'{self.mw.current_dac_voltage:.3f}', 'Rm (kOhm)', f'{float(self.mw.current_selected_res_kohm):.3f}', 'Speed', meta_speed, 'Microstep', str(self.mw.microstep_factor)])
                                writer.writerow(['Cycle', 'Time (s)', 'Relative Distance (mm)', 'ADC Voltage (V)', 'Resistance (kOhm)'])
                            self.mw._log_message(f"CSV header & metadata written to: {self.mw.recording_filepath}")
                    except Exception as e:
                        self.mw._log_message(f"Error creating/writing header: {e}")
                        self.mw.is_actively_recording = False
                        self.mw.is_recording_armed = False
                        if hasattr(self.ui, 'actionRecord'):
                            self.ui.actionRecord.setChecked(False)
                            self.ui.actionRecord.setText("Record Data")
                        if hasattr(self.mw, 'recording_status_indicator'):
                            self.mw.recording_status_indicator.hide()
                        QMessageBox.critical(self.mw, "File Error", f"Could not write header:\n{e}")

                    if self.mw.is_actively_recording and hasattr(self.mw, 'recording_status_indicator'):
                        self.mw.recording_status_indicator.setText("🔴 Recording...")
                        self.mw.recording_status_indicator.setStyleSheet("color: red; margin-left: 10px; font-weight: bold;")
                        self.mw.recording_status_indicator.show()
                        self.serial.send_command("$REC_ON")
                    self.mw._log_message(f"Manual run started. Recording to: {self.mw.recording_filepath if self.mw.is_actively_recording else 'N/A'}")
                else:
                    self.mw._log_message("Manual run started (recording not armed).")

                self.mw._update_controls_state()
            else:
                QMessageBox.critical(self.mw, "Send Error", "Failed to send Manual Run command.")

        except ValueError:
            QMessageBox.warning(self.mw, "Invalid Input", "Invalid number format for Start/End (cm).")
        except Exception as e:
            QMessageBox.critical(self.mw, "Error", f"An unexpected error during manual run start: {e}")