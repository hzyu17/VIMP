import sys
import os
import subprocess
import xml.etree.ElementTree as ET
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QWidget, QFileDialog,
    QMessageBox, QGridLayout, QGroupBox
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QThread, pyqtSignal

file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(file_path)
# config_dir = os.path.abspath(os.path.join(current_dir, '../../configs/vimp/sparse_gh'))
config_dir = os.path.abspath(os.path.join(current_dir, '../../configs/proxkl'))

class RunProgramThread(QThread):
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)
    
    def __init__(self, executable, matlab_script, parent=None):
        super().__init__(parent)
        self.executable = executable
        self.matlab_script = matlab_script
        self._stop_requested = False
        self.processes = []  # Store running subprocesses

    def run(self):
        try:
            # Run the C++ executable
            if self._stop_requested:
                return
            process_exe = subprocess.Popen([self.executable])
            self.processes.append(process_exe)
            # Polling to check process status
            while process_exe.poll() is None:
                if self._stop_requested:
                    process_exe.terminate()
                    return
                time.sleep(0.1)

            if self._stop_requested:
                return

            # Run the MATLAB script
            matlab_command = f"run('{self.matlab_script}')"
            process_matlab = subprocess.Popen(["matlab", "-batch", matlab_command])
            self.processes.append(process_matlab)
            while process_matlab.poll() is None:
                if self._stop_requested:
                    process_matlab.terminate()
                    return
                time.sleep(0.1)
        except Exception as e:
            self.error_signal.emit(str(e))
        finally:
            self.finished_signal.emit()

    def stop(self):
        self._stop_requested = True
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()

class ParameterEditor(QMainWindow):
    def __init__(self):
        super().__init__()

        self.xml_file = None
        self.parameters = {}
        self.original_parameters = {}
        self.run_thread = None  # Background run thread
        self.initUI()

    def initUI(self):
        self.setWindowTitle("XML Parameter Editor and Runner")
        
        # Main Layout
        main_layout = QVBoxLayout()

        # File Selection
        file_layout = QHBoxLayout()
        self.file_label = QLabel("No file selected")
        self.file_button = QPushButton("Select XML File")
        self.file_button.clicked.connect(self.load_xml_file)
        file_layout.addWidget(self.file_label)
        file_layout.addWidget(self.file_button)
        main_layout.addLayout(file_layout)

        # Parameter Grid
        self.param_group = QGroupBox("Parameters")
        self.param_layout = QGridLayout()
        self.param_group.setLayout(self.param_layout)
        main_layout.addWidget(self.param_group)

        # Control Buttons
        control_layout = QHBoxLayout()
        self.run_button = QPushButton("Run Program")
        self.run_button.clicked.connect(self.run_program)
        self.read_parameters_button = QPushButton("Read Parameters")
        self.read_parameters_button.clicked.connect(self.read_parameters)
        self.reset_button = QPushButton("Reset Parameters")
        self.reset_button.clicked.connect(self.reset_parameters)
        self.load_images_button = QPushButton("Load Images")
        self.load_images_button.clicked.connect(self.display_images)
        self.stop_button = QPushButton("Stop Run")
        self.stop_button.clicked.connect(self.stop_run_program)
        self.stop_button.setEnabled(False)  # Initially disabled

        control_layout.addWidget(self.run_button)
        control_layout.addWidget(self.read_parameters_button)
        control_layout.addWidget(self.reset_button)
        control_layout.addWidget(self.load_images_button)
        control_layout.addWidget(self.stop_button)
        main_layout.addLayout(control_layout)

        # Image Display (Horizontal Layout)
        image_layout = QHBoxLayout()
        self.image_label_1 = QLabel("Image 1 will appear here")
        self.image_label_2 = QLabel("Image 2 will appear here")
        image_layout.addWidget(self.image_label_1)
        image_layout.addWidget(self.image_label_2)
        main_layout.addLayout(image_layout)

        # Set Central Widget
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def load_xml_file(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        file_name, _ = QFileDialog.getOpenFileName(self, "Select XML File", config_dir, "XML Files (*.xml)", options=options)

        if file_name:
            self.xml_file = file_name
            self.file_label.setText(os.path.basename(file_name))
            self.read_parameters()

    def read_parameters(self):
        self.parameters.clear()
        self.original_parameters.clear()
        try:
            # Read the XML file and extract the <Commons> section
            with open(self.xml_file, 'r') as file:
                lines = file.readlines()

            commons_data = ""
            in_commons = False
            for line in lines:
                if "<Commons>" in line:
                    in_commons = True
                if in_commons:
                    commons_data += line
                if "</Commons>" in line:
                    break

            if not commons_data:
                QMessageBox.critical(self, "Error", "No <Commons> section found in the XML file.")
                return

            commons_element = ET.fromstring(commons_data)

            # Clear previous layout
            for i in reversed(range(self.param_layout.count())):
                self.param_layout.itemAt(i).widget().setParent(None)

            row = 0
            for child in commons_element:
                name = child.tag
                value = child.text
                self.parameters[name] = value
                self.original_parameters[name] = value

                self.param_layout.addWidget(QLabel(name), row, 0)
                line_edit = QLineEdit(value)
                line_edit.setObjectName(name)
                self.param_layout.addWidget(line_edit, row, 1)
                row += 1
        except ET.ParseError as e:
            QMessageBox.critical(self, "Error", f"Failed to parse <Commons>: {e}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {e}")

    def save_xml_file(self):
        if not self.xml_file:
            QMessageBox.warning(self, "Error", "No XML file loaded.")
            return

        try:
            for i in range(self.param_layout.rowCount()):
                label_widget = self.param_layout.itemAtPosition(i, 0).widget()
                line_edit_widget = self.param_layout.itemAtPosition(i, 1).widget()
                if label_widget and line_edit_widget:
                    key = label_widget.text()
                    value = line_edit_widget.text()
                    self.parameters[key] = value

            with open(self.xml_file, 'r') as file:
                lines = file.readlines()

            new_lines = []
            in_commons = False
            for line in lines:
                if "<Commons>" in line:
                    in_commons = True
                    new_lines.append(line)
                    for name, value in self.parameters.items():
                        new_lines.append(f"    <{name}>{value}</{name}>\n")
                    continue
                if "</Commons>" in line and in_commons:
                    in_commons = False
                    new_lines.append(line)
                    continue
                if not in_commons:
                    new_lines.append(line)

            with open(self.xml_file, 'w') as file:
                file.writelines(new_lines)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save XML file: {e}")
            print(f"Error: {e}")

    def reset_parameters(self):
        for name, value in self.original_parameters.items():
            line_edit = self.param_group.findChild(QLineEdit, name)
            if line_edit:
                line_edit.setText(value)

    def run_program(self):
        if not self.xml_file:
            QMessageBox.warning(self, "Error", "No XML file loaded.")
            return

        # Save current parameters
        self.save_xml_file()

        # executable = "/home/zinuo/VIMP/build/src/gvimp/gvi_Quadrotor_spgh"
        # matlab_script = "/home/zinuo/VIMP/matlab_helpers/GVIMP-examples/2d_Quad/planarQuad_morestates.m"

        executable = "/home/zinuo/VIMP/build/src/gvimp/proxkl_Quadrotor_spgh"
        matlab_script = "/home/zinuo/VIMP/matlab_helpers/ProxKL-examples/2d_Quad/planarQuad_SingleObs.m"

        # Disable run button and enable stop button
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        # Create and start background thread
        self.run_thread = RunProgramThread(executable, matlab_script)
        self.run_thread.finished_signal.connect(self.on_run_finished)
        self.run_thread.error_signal.connect(lambda err: QMessageBox.critical(self, "Error", f"Error running program: {err}"))
        self.run_thread.start()

    def on_run_finished(self):
        # Restore button states
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.run_thread = None
        # Load generated images after program finishes running
        self.display_images()

    def stop_run_program(self):
        if self.run_thread:
            self.run_thread.stop()
            # Optionally wait for thread to finish before restoring state
            self.run_thread.wait()
            self.on_run_finished()
            print("Program has been stopped")  # Print stop message to the command line

    def display_images(self):
        # Paths to the generated images
        # image_path_1 = "/home/zinuo/VIMP/matlab_helpers/GVIMP-examples/2d_Quad/case2_300_states/output_figure_1.png"
        # image_path_2 = "/home/zinuo/VIMP/matlab_helpers/GVIMP-examples/2d_Quad/case2_300_states/output_figure_2.png"

        image_path_1 = "/home/zinuo/VIMP/matlab_helpers/ProxKL-examples/2d_Quad/case2/output_figure_1.png"
        image_path_2 = "/home/zinuo/VIMP/matlab_helpers/ProxKL-examples/2d_Quad/case2/output_figure_2.png"

        max_width = 800
        max_height = 600

        if os.path.exists(image_path_1):
            pixmap1 = QPixmap(image_path_1)
            pixmap1 = pixmap1.scaled(max_width, max_height, aspectRatioMode=1)
            self.image_label_1.setPixmap(pixmap1)
            self.image_label_1.setScaledContents(True)

        if os.path.exists(image_path_2):
            pixmap2 = QPixmap(image_path_2)
            pixmap2 = pixmap2.scaled(max_width, max_height, aspectRatioMode=1)
            self.image_label_2.setPixmap(pixmap2)
            self.image_label_2.setScaledContents(True)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    editor = ParameterEditor()
    editor.show()
    sys.exit(app.exec_())
