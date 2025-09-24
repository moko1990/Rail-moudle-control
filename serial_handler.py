# filename: serial_handler.py
# Handles serial communication with the Arduino device in a separate thread.
# Version: 2.1 (clean thread shutdown, buffer guard, optional fast DTR reset, robust connect/disconnect)

import time
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal, QThread, Slot
import threading


class SerialReader(QObject):
    """
    Worker QObject that runs in a separate QThread to continuously read
    data from the serial port without blocking the main UI thread.
    Emits a signal for each line of data received.
    """
    data_received = Signal(str)    # Signal emitted when a line of data is received
    error_occurred = Signal(str)   # Signal emitted when a serial error occurs during reading
    finished = Signal()            # Signal emitted when the run loop finishes

    def __init__(self, serial_instance):
        """
        Args:
            serial_instance: An already opened pySerial object.
        """
        super().__init__()
        self.serial = serial_instance
        self._running = False  # Flag to control the reading loop

    @Slot()
    def run(self):
        """
        Main loop: reads chunks from serial, splits by newline, emits lines.
        Buffered approach avoids partial lines and reduces blocking.
        """
        self._running = True
        print("Serial reader thread started.")
        buf = bytearray()

        # Guards
        MAX_BUF = 65536   # 64 KB max buffer if '\n' never arrives (prevents RAM runaway)
        MAX_LINE = 4096   # 4 KB max logical line length (safety for malformed frames)

        try:
            while self._running and self.serial and self.serial.is_open:
                try:
                    n_wait = self.serial.in_waiting
                    # Read whatever is available, or at least 1 byte (honors timeout)
                    chunk = self.serial.read(n_wait or 1)
                    if chunk:
                        buf.extend(chunk)

                        # Guard against runaway buffer if '\n' never arrives
                        if len(buf) > MAX_BUF:
                            last_nl = buf.rfind(b'\n')
                            if last_nl != -1:
                                del buf[:last_nl + 1]
                            else:
                                buf.clear()

                        # Extract complete lines separated by '\n'
                        while True:
                            nl_idx = buf.find(b'\n')
                            if nl_idx == -1:
                                break

                            line_bytes = buf[:nl_idx]     # up to before '\n'
                            del buf[:nl_idx + 1]          # remove through '\n'

                            # Limit line length (keeps tail)
                            if len(line_bytes) > MAX_LINE:
                                line_bytes = line_bytes[-MAX_LINE:]

                            # Decode ASCII, ignore invalid bytes; strip '\r' and spaces
                            line = line_bytes.decode('ascii', errors='ignore').strip('\r').strip()
                            if line:
                                self.data_received.emit(line)
                    else:
                        # Tiny sleep to avoid busy-waiting
                        time.sleep(0.002)

                except serial.SerialException as e:
                    if self._running:
                        msg = f"Serial Error in reader: {e}"
                        self.error_occurred.emit(msg)
                        print(msg)
                    break
                except Exception as e:
                    if self._running:
                        msg = f"Unexpected Reader Error: {e}"
                        self.error_occurred.emit(msg)
                        print(msg)
                    break

        finally:
            self._running = False
            print("Serial reader thread finished.")
            self.finished.emit()

    def stop(self):
        """
        Requests the reading loop to stop on next iteration.
        """
        print("Requesting serial reader thread to stop...")
        self._running = False


class SerialHandler(QObject):
    """
    Manages overall serial communication including port listing,
    connection, disconnection, and sending commands.
    Uses a SerialReader worker in a separate QThread for non-blocking data reception.
    """
    # Signals to communicate with the main UI thread (MainWindow)
    connected = Signal()             # Emitted when connection is successful
    disconnected = Signal()          # Emitted when disconnection is complete or connection lost
    ports_updated = Signal(list)     # Emitted with a list of available serial port names
    data_received = Signal(str)      # Forwarded from SerialReader when new data arrives
    error_occurred = Signal(str)     # Emitted for connection errors or forwarded from SerialReader

    def __init__(self, baud_rate=250000, use_dtr_reset=True):
        """
        Args:
            baud_rate (int): The baud rate for serial communication.
            use_dtr_reset (bool): Toggle fast DTR reset on connect (reduces wait time on most boards).
        """
        super().__init__()
        self.baud_rate = baud_rate
        self.use_dtr_reset = bool(use_dtr_reset)

        self.serial_port = None       # pySerial instance
        self.reader_thread = None     # QThread for the reader
        self.reader_worker = None     # SerialReader instance
        self._is_connected = False    # Internal connection status flag
        self._write_lock = threading.Lock()  # Thread-safe write

    @Slot()
    def list_ports(self):
        """
        Finds available serial ports on the system.
        Emits the ports_updated signal with a list of port names.
        Returns:
            list: A list of available serial port names.
        """
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            print(f"Available serial ports: {ports}")
            self.ports_updated.emit(ports)
            return ports
        except Exception as e:
            error_msg = f"Error listing serial ports: {e}"
            self.error_occurred.emit(error_msg)
            print(error_msg)
            self.ports_updated.emit([])  # Emit empty list on error
            return []

    def _stop_reader_thread_if_running(self):
        """Gracefully stop an existing reader thread if present."""
        if self.reader_thread and self.reader_thread.isRunning():
            print("Stopping existing serial reader worker...")
            if self.reader_worker:
                self.reader_worker.stop()  # Ask loop to stop

            print("Quitting serial reader thread's event loop...")
            self.reader_thread.quit()      # Ask the thread's event loop to exit

            print("Waiting for serial reader thread to finish...")
            if not self.reader_thread.wait(1500):  # Wait up to 1.5 seconds
                print("Warning: Serial reader thread did not finish gracefully. Terminating.")
                self.reader_thread.terminate()
                self.reader_thread.wait()
            print("Existing serial reader thread stopped.")

        # Clear references (thread/worker get deleted later via deleteLater as well)
        self.reader_thread = None
        self.reader_worker = None

    @Slot(str)
    def connect(self, port_name):
        """
        Attempts to connect to the specified serial port.
        If successful, starts the SerialReader thread.
        Args:
            port_name (str): e.g., "COM5" or "/dev/ttyUSB0"
        Returns:
            bool: True on success, False otherwise.
        """
        if self.serial_port and self.serial_port.is_open:
            print("Already connected to a port.")
            return True

        if not port_name:
            self.error_occurred.emit("No port selected for connection.")
            return False

        try:
            # Ensure previous reader (if any) is stopped
            self._stop_reader_thread_if_running()

            print(f"Attempting to connect to '{port_name}' at {self.baud_rate} baud...")
            # Set reasonable timeouts to avoid long blocking
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=self.baud_rate,
                timeout=0.1,
                write_timeout=0.25
            )

            # Flush buffers after opening port
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
            except Exception:
                pass

            # Optional: fast DTR reset (reduces wait time on many boards)
            if self.use_dtr_reset:
                try:
                    self.serial_port.dtr = False
                    time.sleep(0.05)
                    self.serial_port.dtr = True
                except Exception:
                    pass
                wait_s = 1.2
            else:
                # Conservative wait time if no DTR reset
                wait_s = 1.8

            time.sleep(wait_s)

            try:
                self.serial_port.reset_input_buffer()
            except Exception:
                pass

            if self.serial_port.is_open:
                self._is_connected = True
                print(f"Successfully connected to {port_name}.")

                # Setup reader worker and thread
                self.reader_worker = SerialReader(self.serial_port)
                self.reader_thread = QThread()
                self.reader_worker.moveToThread(self.reader_thread)

                # Wire signals
                self.reader_worker.data_received.connect(self.data_received)     # Forward data
                self.reader_worker.error_occurred.connect(self._handle_reader_error)
                self.reader_worker.finished.connect(self._on_reader_finished)

                # Clean shutdown of thread after worker.run() ends
                self.reader_worker.finished.connect(self.reader_thread.quit)
                self.reader_thread.finished.connect(self.reader_worker.deleteLater)
                self.reader_thread.finished.connect(self.reader_thread.deleteLater)

                self.reader_thread.started.connect(self.reader_worker.run)
                self.reader_thread.start()
                print("Serial reader thread initiated.")

                # Notify UI
                self.connected.emit()
                return True
            else:
                error_msg = f"Failed to open port {port_name}."
                self.error_occurred.emit(error_msg)
                self._is_connected = False
                self.serial_port = None
                return False

        except serial.SerialException as e:
            error_msg = f"Serial Connection Error on '{port_name}': {e}"
            self.error_occurred.emit(error_msg)
            print(error_msg)
            self._is_connected = False
            self.serial_port = None
            return False
        except Exception as e:
            error_msg = f"Unexpected Error during connection to '{port_name}': {e}"
            self.error_occurred.emit(error_msg)
            print(error_msg)
            self._is_connected = False
            self.serial_port = None
            return False

    @Slot()
    def disconnect(self):
        """
        Closes the serial port and gracefully stops the SerialReader thread.
        Emits the disconnected signal.
        """
        if not self._is_connected and not self.serial_port:
            print("Already disconnected or no port was open.")
            return

        print("Disconnect requested.")

        # 1) Stop reader thread
        self._stop_reader_thread_if_running()

        # 2) Close the serial port
        if self.serial_port and self.serial_port.is_open:
            port_name_to_close = self.serial_port.name
            try:
                self.serial_port.close()
                print(f"Serial port {port_name_to_close} closed.")
            except Exception as e:
                error_msg = f"Error closing serial port {port_name_to_close}: {e}"
                self.error_occurred.emit(error_msg)
                print(error_msg)

        # 3) Reset state
        self.serial_port = None
        prev_connected = self._is_connected
        self._is_connected = False

        # 4) Signal UI (emit only if it was connected)
        if prev_connected:
            self.disconnected.emit()
        print("Disconnection process complete.")

    @Slot(str)
    def send_command(self, command):
        """
        Sends a command string to the connected Arduino.
        Appends newline automatically if missing.
        Args:
            command (str): The command string to send.
        Returns:
            bool: True if the command was sent successfully, False otherwise.
        """
        if self.serial_port and self.serial_port.is_open:
            try:
                if not command.endswith('\n'):
                    command += '\n'
                # Thread-safe write
                with self._write_lock:
                    self.serial_port.write(command.encode('ascii', errors='ignore'))
                print(f"Sent: {command.strip()}")
                return True
            except serial.SerialTimeoutException as e:
                error_msg = f"Send Timeout Error: {e}"
                self.error_occurred.emit(error_msg)
                print(error_msg)
                return False
            except serial.SerialException as e:
                error_msg = f"Serial Send Error: {e}"
                self.error_occurred.emit(error_msg)
                print(error_msg)
                return False
            except Exception as e:
                error_msg = f"Unexpected Send Error: {e}"
                self.error_occurred.emit(error_msg)
                print(error_msg)
                return False
        else:
            msg = "Cannot send command: Not connected."
            self.error_occurred.emit(msg)
            print(msg)
            return False

    def is_connected(self) -> bool:
        """Returns the current connection status."""
        return self._is_connected

    @Slot(str)
    def _handle_reader_error(self, error_msg):
        """Handles errors emitted by the SerialReader worker."""
        print(f"SerialHandler received reader error: {error_msg}")
        self.error_occurred.emit(error_msg)
        # Optionally, could trigger auto-disconnect here if needed

    @Slot()
    def _on_reader_finished(self):
        """
        Called when the SerialReader thread's finished signal is emitted.
        If still marked connected, assume unexpected stop and emit disconnected.
        """
        print("SerialHandler notified that reader thread finished.")

        # Fail-safe: if thread is still running, request quit
        try:
            if self.reader_thread and self.reader_thread.isRunning():
                self.reader_thread.quit()
        except Exception:
            pass

        if self._is_connected:
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            self._is_connected = False
            self.disconnected.emit()