import sys
import os
import json
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QPushButton, QFileDialog, QScrollArea, QMessageBox)
from scipy.ndimage import distance_transform_edt as bwdist

# Suppose libplanar_sdf is in your build directory
file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(file_path)
build_dir = os.path.abspath(os.path.join(current_dir, '../sdf_robot/build'))
map_dir = os.path.abspath(os.path.join(current_dir, '../sdf_robot/map/planar'))
sys.path.append(build_dir)

import libplanar_sdf

class map2d:
    """
    2D map object that holds the binary map and its signed distance field.
    """
    def __init__(self, origin, cell_size, map_width, map_height, map_name='defaultmap') -> None:
        self.map_name = map_name
        self.origin = origin
        self._cell_size = cell_size
        self.map_width = map_width
        self.map_height = map_height
        self._map = np.zeros((map_height, map_width), dtype=np.float64)
        self._field = np.zeros((map_height, map_width), dtype=np.float64)

    def add_box_xy(self, xmin, ymin, shape):
        xmax = xmin + shape[0]
        ymax = ymin + shape[1]
        xmin_idx = int((xmin - self.origin[0]) / self._cell_size)
        xmax_idx = int((xmax - self.origin[0]) / self._cell_size)
        ymin_idx = int((ymin - self.origin[1]) / self._cell_size)
        ymax_idx = int((ymax - self.origin[1]) / self._cell_size)
        self._map[ymin_idx:ymax_idx, xmin_idx:xmax_idx] = 1.0
        self.generate_SDField()

    def generate_SDField(self):
        inverse_map = 1.0 - self._map
        inside_dist = bwdist(self._map)
        outside_dist = bwdist(inverse_map)
        self._field = (outside_dist - inside_dist) * self._cell_size

    def save_map(self, map_name, field_name):
        # Save the map and its field as CSV for other programs
        np.savetxt(map_name, self._map, delimiter=',')
        np.savetxt(field_name, self._field, delimiter=',')

    def get_map(self):
        return self._map
    
    def get_field(self):
        return self._field

    def draw_map(self, fig, ax, plot=True, labels=False):
        cmap = plt.cm.colors.ListedColormap(['white', 'black'])
        ax.imshow(self._map, cmap=cmap, interpolation='nearest', origin='lower',
                  extent=[self.origin[0], self.origin[0] + self.map_width * self._cell_size,
                          self.origin[1], self.origin[1] + self.map_height * self._cell_size])
        if labels:
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_xlim([self.origin[0], self.origin[0] + self.map_width * self._cell_size])
            ax.set_ylim([self.origin[1], self.origin[1] + self.map_height * self._cell_size])
            ax.set_title('Obstacle Environment')
        if plot:
            plt.show()
        return fig, ax

    def get_cell_size(self):
        return self._cell_size
    
    def get_width(self):
        return self.map_width
    
    def get_height(self):
        return self.map_height
    
    def get_origin(self):
        return self.origin


class MapEditorApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Planar Map Editor")
        self.map_obj = None
        self.obstacle_rows = []

        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        self.main_layout = QVBoxLayout()
        main_widget.setLayout(self.main_layout)

        # Matplotlib canvas
        self.fig = Figure(figsize=(6,6))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        self.main_layout.addWidget(self.canvas)

        # Parameter area
        self.param_widget = QWidget()
        self.param_layout = QHBoxLayout()
        self.param_widget.setLayout(self.param_layout)
        self.main_layout.addWidget(self.param_widget)
        self.create_parameter_widgets()
        self.set_default_parameters()

        # Obstacle area with scroll
        self.obstacle_scroll = QScrollArea()
        self.obstacle_scroll.setWidgetResizable(True)
        self.obstacle_container = QWidget()
        self.obstacle_layout = QVBoxLayout()
        self.obstacle_layout.setSpacing(10)
        self.obstacle_layout.setContentsMargins(0,0,0,0)
        self.obstacle_container.setLayout(self.obstacle_layout)
        self.obstacle_scroll.setWidget(self.obstacle_container)
        self.obstacle_scroll.setFixedHeight(0)
        self.main_layout.addWidget(self.obstacle_scroll)

        # Button to add obstacle
        self.add_obstacle_btn = QPushButton("Add Obstacle")
        self.add_obstacle_btn.clicked.connect(self.add_obstacle_row)
        self.main_layout.addWidget(self.add_obstacle_btn)

        # Action buttons
        self.button_widget = QWidget()
        self.button_layout = QHBoxLayout()
        self.button_widget.setLayout(self.button_layout)
        self.main_layout.addWidget(self.button_widget)
        self.create_action_buttons()

    def create_parameter_widgets(self):
        labels = ["Map Name", "Origin X", "Origin Y", "Cell Size", "Width", "Height"]
        self.param_entries = {}
        for label_text in labels:
            lbl = QLabel(label_text)
            le = QLineEdit()
            self.param_layout.addWidget(lbl)
            self.param_layout.addWidget(le)
            self.param_entries[label_text] = le

    def set_default_parameters(self):
        defaults = {
            "Map Name": "ObstacleMap",
            "Origin X": "-20.0",
            "Origin Y": "-20.0",
            "Cell Size": "0.1",
            "Width": "700",
            "Height": "700"
        }
        for k, v in defaults.items():
            self.param_entries[k].setText(v)

    def add_obstacle_row(self):
        row_widget = QWidget()
        row_layout = QHBoxLayout()
        row_layout.setSpacing(0)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_widget.setLayout(row_layout)
        row_widget.setFixedHeight(45)

        # Generate row number based on current row count
        row_number = len(self.obstacle_rows) + 1
        obs_label = QLabel(f"Obs{row_number}:")
        obs_label.setContentsMargins(2, 2, 2, 2)
        row_layout.addWidget(obs_label)

        entry_widgets = {}
        for field in ["xmin", "ymin", "width", "height"]:
            lbl = QLabel(field)
            lbl.setContentsMargins(2, 2, 2, 2)
            le = QLineEdit()
            le.setFixedHeight(40)
            row_layout.addWidget(lbl)
            row_layout.addWidget(le)
            entry_widgets[field] = le

        del_btn = QPushButton("Delete")
        del_btn.setFixedHeight(40)
        del_btn.clicked.connect(lambda: self.delete_obstacle_row(row_widget))
        row_layout.addWidget(del_btn)

        self.obstacle_layout.addWidget(row_widget)
        self.obstacle_rows.append((row_widget, entry_widgets))
        self.update_obstacle_area_height()

    def delete_obstacle_row(self, row_widget):
        for i, (widget, entries) in enumerate(self.obstacle_rows):
            if widget == row_widget:
                self.obstacle_layout.removeWidget(widget)
                widget.deleteLater()
                self.obstacle_rows.pop(i)
                break
        # Update the labels to reflect the new row numbers
        self.update_obstacle_labels()
        self.update_obstacle_area_height()

    def update_obstacle_labels(self):
        for index, (widget, entries) in enumerate(self.obstacle_rows, start=1):
            layout = widget.layout()
            label_widget = layout.itemAt(0).widget()
            label_widget.setText(f"Obs{index}:")

    def update_obstacle_area_height(self):
        row_count = len(self.obstacle_rows)
        row_height = 45
        spacing = 10
        if row_count == 0:
            new_height = 0
        elif row_count < 5:
            new_height = row_count * row_height + (row_count - 1) * spacing
        else:
            new_height = 5 * row_height + 4 * spacing
        self.obstacle_scroll.setFixedHeight(new_height)

    def create_action_buttons(self):
        show_map_btn = QPushButton("Show Map")
        show_map_btn.clicked.connect(self.show_map)
        self.button_layout.addWidget(show_map_btn)

        save_map_btn = QPushButton("Save Map")
        save_map_btn.clicked.connect(self.save_map)
        self.button_layout.addWidget(save_map_btn)

        load_map_btn = QPushButton("Load Map")
        load_map_btn.clicked.connect(self.load_map)
        self.button_layout.addWidget(load_map_btn)

        reset_btn = QPushButton("Reset Map")
        reset_btn.clicked.connect(self.reset_map)
        self.button_layout.addWidget(reset_btn)

        export_btn = QPushButton("Export Image")
        export_btn.clicked.connect(self.export_image)
        self.button_layout.addWidget(export_btn)

    def show_map(self):
        """Create map2d object from GUI parameters and obstacles, then display it."""
        try:
            map_name = self.param_entries["Map Name"].text()
            origin_x = float(self.param_entries["Origin X"].text())
            origin_y = float(self.param_entries["Origin Y"].text())
            cell_size = float(self.param_entries["Cell Size"].text())
            width = int(self.param_entries["Width"].text())
            height = int(self.param_entries["Height"].text())
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Please enter valid numerical parameters!")
            return
        
        origin = np.array([origin_x, origin_y], dtype=np.float64)
        m = map2d(origin, cell_size, width, height, map_name)

        for (widget, entries) in self.obstacle_rows:
            try:
                xmin = float(entries["xmin"].text())
                ymin = float(entries["ymin"].text())
                obs_w = float(entries["width"].text())
                obs_h = float(entries["height"].text())
            except ValueError:
                QMessageBox.critical(self, "Input Error", "Invalid obstacle parameters!")
                return
            m.add_box_xy(xmin, ymin, [obs_w, obs_h])

        self.ax.clear()
        m.draw_map(self.fig, self.ax, plot=False, labels=True)
        self.canvas.draw()
        self.map_obj = m

    def save_map(self):
        """
        Save CSV for other programs, but only store parameters + obstacles in JSON.
        We do NOT store the entire map data in JSON. We'll rebuild from obstacles if needed.
        """
        if self.map_obj is None:
            QMessageBox.critical(self, "Error", "Please generate the map before saving!")
            return
        self.show_map()  # Ensure map data is up to date

        default_full_path = os.path.join(map_dir, f"{self.map_obj.map_name}.csv")
        file_path, _ = QFileDialog.getSaveFileName(self, "Save Map", default_full_path, "CSV files (*.csv)")
        if not file_path:
            return

        # 1) Save CSV data for other programs
        field_file = file_path.replace(".csv", "_field.csv")
        self.map_obj.save_map(file_path, field_file)

        # 2) Save JSON with parameters + obstacles, but NOT the entire map data
        obstacles = []
        for (_, entries) in self.obstacle_rows:
            try:
                xmin = float(entries["xmin"].text())
                ymin = float(entries["ymin"].text())
                w = float(entries["width"].text())
                h = float(entries["height"].text())
                obstacles.append({"xmin": xmin, "ymin": ymin, "width": w, "height": h})
            except ValueError:
                continue

        params = {
            "map_name": self.map_obj.map_name,
            "origin": self.map_obj.get_origin().tolist(),
            "cell_size": self.map_obj.get_cell_size(),
            "map_width": self.map_obj.get_width(),
            "map_height": self.map_obj.get_height(),
            "obstacles": obstacles
            # We do NOT store "map_data"
        }
        param_file = file_path.replace(".csv", "_params.json")
        with open(param_file, "w") as f:
            json.dump(params, f, indent=4)

        QMessageBox.information(
            self,
            "Save Map",
            f"CSV saved:\n{file_path}\n{field_file}\nParameters:\n{param_file}"
        )

    def load_map(self):
        """
        Load ONLY from JSON. We do NOT parse CSV here. We'll rebuild the map from scratch
        by reading the obstacles in JSON and re-adding them to an empty map2d.
        """
        file_filter = "JSON files (*.json);;All Files (*.*)"
        file_path, _ = QFileDialog.getOpenFileName(self, "Load Map (JSON)", map_dir, file_filter)
        if not file_path:
            return

        try:
            with open(file_path, "r") as f:
                params = json.load(f)

            # Update GUI fields
            self.param_entries["Map Name"].setText(params["map_name"])
            self.param_entries["Origin X"].setText(str(params["origin"][0]))
            self.param_entries["Origin Y"].setText(str(params["origin"][1]))
            self.param_entries["Cell Size"].setText(str(params["cell_size"]))
            self.param_entries["Width"].setText(str(params["map_width"]))
            self.param_entries["Height"].setText(str(params["map_height"]))

            # Clear existing obstacles
            for (widget, _) in self.obstacle_rows:
                self.obstacle_layout.removeWidget(widget)
                widget.deleteLater()
            self.obstacle_rows.clear()
            self.update_obstacle_area_height()

            # Restore obstacles from JSON
            for obs in params.get("obstacles", []):
                self.add_obstacle_row()
                _, entries = self.obstacle_rows[-1]
                entries["xmin"].setText(str(obs["xmin"]))
                entries["ymin"].setText(str(obs["ymin"]))
                entries["width"].setText(str(obs["width"]))
                entries["height"].setText(str(obs["height"]))

            # Now rebuild the map from scratch using these parameters + obstacles
            # It's effectively the same logic as show_map()
            map_name = params["map_name"]
            origin_x = params["origin"][0]
            origin_y = params["origin"][1]
            cell_size = params["cell_size"]
            width = params["map_width"]
            height = params["map_height"]

            origin = np.array([origin_x, origin_y], dtype=np.float64)
            m = map2d(origin, cell_size, width, height, map_name)
            # Add each obstacle
            for obs in params.get("obstacles", []):
                m.add_box_xy(obs["xmin"], obs["ymin"], [obs["width"], obs["height"]])

            self.ax.clear()
            m.draw_map(self.fig, self.ax, plot=False, labels=True)
            self.canvas.draw()
            self.map_obj = m

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load JSON:\n{e}")

    def reset_map(self):
        self.set_default_parameters()
        for (widget, _) in self.obstacle_rows:
            self.obstacle_layout.removeWidget(widget)
            widget.deleteLater()
        self.obstacle_rows.clear()
        self.update_obstacle_area_height()
        self.map_obj = None
        self.ax.clear()
        self.canvas.draw()

    def export_image(self):
        if self.map_obj is None:
            QMessageBox.critical(self, "Error", "Please generate the map before exporting an image!")
            return
        file_path, _ = QFileDialog.getSaveFileName(self, "Export Image", map_dir, "PNG files (*.png)")
        if file_path:
            self.fig.savefig(file_path)
            QMessageBox.information(self, "Export Image", f"Image exported:\n{file_path}")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    editor = MapEditorApp()
    editor.resize(800, 900)
    editor.show()
    sys.exit(app.exec_())
