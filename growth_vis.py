import sys
import vtkmodules.all as vtk
import open3d as o3d
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import os
from PIL import Image, ImageDraw, ImageFont
import io
from datetime import datetime
import yaml
growth_yaml_path = 'config/growth_vis.yaml'

def load_yaml_config(config_path="config.yaml"):
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config
    except Exception as e:
        print(f"Error loading YAML config: {e}")
        return None

def filter_cloud(points, x_min, x_max, y_min, y_max, z_min, z_max):
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    mask &= (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
    mask &= (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
    in_range_points = points[mask]
    out_range_points = points[~mask]
    return in_range_points, out_range_points

def create_vector2id_map(points, cell_size):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    vector2id = {}
    for idx in range(len(points)):
        key = int(points[idx, 0] // cell_size), int(points[idx, 1] // cell_size)
        if key not in vector2id:
            vector2id[key] = {'points': [], 'max_height': z[idx]}
        vector2id[key]['points'].append(points[idx])
        if z[idx] > vector2id[key]['max_height']:
            vector2id[key]['max_height'] = z[idx]
    return vector2id

def compute_growth(base, reference, cell_size, ranges, growth_mu, non_growth_color):
    x_min, x_max, y_min, y_max, z_min, z_max = ranges
    base_filtered, base_non_filtered = filter_cloud(base, x_min, x_max, y_min, y_max, z_min, z_max)
    reference_filtered, reference_non_filtered = filter_cloud(reference, x_min, x_max, y_min, y_max, z_min, z_max)
    base_vector2id = create_vector2id_map(base_filtered, cell_size)
    reference_vector2id = create_vector2id_map(reference_filtered, cell_size)

    growth = {}
    for key in base_vector2id.keys():
        if key in reference_vector2id:
            base_max_height = base_vector2id[key]['max_height']
            reference_max_height = reference_vector2id[key]['max_height']
            growth[key] = reference_max_height - base_max_height
            if growth[key] < 0.25:
                growth[key] = np.nan
        else:
            growth[key] = np.nan

    com_points = []
    colors = []
    growth_values = []

    for key in growth.keys():
        if not np.isnan(growth[key]):
            points = base_vector2id[key]['points'] + reference_vector2id[key]['points']
            com_points.extend(points)
            growth_values.extend([growth[key]] * len(points))

    com_points.extend(base_non_filtered)
    com_points.extend(reference_non_filtered)
    growth_values.extend([np.nan] * (len(base_non_filtered) + len(reference_non_filtered)))

    com_points = np.array(com_points)

    color_map = {'white': [1.0, 1.0, 1.0], 'gray': [0.5, 0.5, 0.5], 'black': [0.0, 0.0, 0.0]}
    non_growth_rgb = color_map.get(non_growth_color.lower(), [1.0, 1.0, 1.0])

    colors = []
    valid_growth = np.array([g for g in growth_values if not np.isnan(g)])
    if len(valid_growth) > 0:
        growth_min = valid_growth.min()
        growth_max = valid_growth.max()
        growth_mean = np.nanmean(np.array(list(growth.values())))
        for g in growth_values:
            if np.isnan(g):
                colors.append(non_growth_rgb)
            else:
                if growth_min <= g < (growth_mean - growth_mu):
                    colors.append([1.0, 0.0, 0.0])
                elif (growth_mean - growth_mu) <= g <= (growth_mean + growth_mu):
                    colors.append([0.0, 0.0, 1.0])
                else:
                    colors.append([0.0, 1.0, 0.0])
    else:
        colors = [non_growth_rgb] * len(com_points)

    return com_points, growth_values, colors, growth, valid_growth

class PointCloudGrowthApp(QtWidgets.QMainWindow):
    def __init__(self, pointcloud_files, cell_size, ranges, point_size, growth_mu, gui_background, non_growth_color):
        super().__init__()
        self.setWindowTitle("Point Cloud Growth Viewer")
        self.pointcloud_files = pointcloud_files
        self.cell_size = cell_size
        self.ranges = ranges
        self.point_size = point_size
        self.growth_mu = growth_mu
        self.non_growth_color = non_growth_color
        self.current_reference = pointcloud_files[1] if len(pointcloud_files) > 1 else pointcloud_files[0]
        self.valid_cells = None
        self.growth_data = {}
        self.valid_growth_values = []

        try:
            icon_path = "logo.png"
            self.setWindowIcon(QtGui.QIcon(icon_path))
        except Exception as e:
            print(f"Failed to set window icon: {e}")

        base_pcd = o3d.io.read_point_cloud(pointcloud_files[0])
        base_points = np.asarray(base_pcd.points)
        for file in pointcloud_files[1:]:
            ref_pcd = o3d.io.read_point_cloud(file)
            com_points, growth_values, colors, growth, valid_growth = compute_growth(
                base_points, np.asarray(ref_pcd.points), cell_size, ranges, growth_mu, non_growth_color
            )
            self.growth_data[file] = {
                'com_points': com_points,
                'growth_values': growth_values,
                'colors': colors,
                'growth': growth,
                'valid_growth': valid_growth
            }
            self.valid_growth_values.extend(valid_growth)

        last_pcd = o3d.io.read_point_cloud(pointcloud_files[-1])
        _, _, _, self.valid_cells, _ = compute_growth(
            base_points, np.asarray(last_pcd.points), cell_size, ranges, growth_mu, non_growth_color
        )

        self.renderer = vtk.vtkRenderer()
        bg_color_map = {'white': [1.0, 1.0, 1.0], 'black': [0.0, 0.0, 0.0]}
        bg_rgb = bg_color_map.get(gui_background.lower(), [0.1, 0.1, 0.1])
        self.renderer.SetBackground(*bg_rgb)
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)

        interactor_style = vtk.vtkInteractorStyleTrackballCamera()
        interactor_style.SetMotionFactor(5.5)
        self.vtk_widget.SetInteractorStyle(interactor_style)

        self.actors = {}
        self.update_point_clouds()

        sidebar_layout = QtWidgets.QVBoxLayout()
        base_label = QtWidgets.QLabel(f"Base: {os.path.basename(pointcloud_files[0])}")
        base_label.setStyleSheet("font-weight: bold; color: black;")
        sidebar_layout.addWidget(base_label)

        for file in pointcloud_files[1:]:
            button = QtWidgets.QPushButton(os.path.basename(file))
            button.setCheckable(True)
            button.setChecked(file == self.current_reference)
            button.clicked.connect(lambda checked, f=file: self.select_reference(f))
            sidebar_layout.addWidget(button)

        sidebar_layout.addStretch()
        sidebar_widget = QtWidgets.QWidget()
        sidebar_widget.setLayout(sidebar_layout)

        self.colorbar_container = QtWidgets.QWidget()
        self.colorbar_container.setLayout(QtWidgets.QVBoxLayout())
        self.update_colorbar()

        self.splitter = QtWidgets.QSplitter()
        self.splitter.addWidget(sidebar_widget)
        self.splitter.addWidget(self.vtk_widget)
        self.splitter.addWidget(self.colorbar_container)
        screen_width = QtWidgets.QApplication.primaryScreen().size().width()
        self.splitter.setSizes([int(150 * screen_width / 1920), int(600 * screen_width / 1920), int(300 * screen_width / 1920)])

        self.setCentralWidget(self.splitter)
        self.show()
        self.vtk_widget.Initialize()
        self.vtk_widget.Start()
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def create_colorbar(self):
        colorbar_widget = QtWidgets.QWidget()
        screen_width = QtWidgets.QApplication.primaryScreen().size().width()
        scale_factor = screen_width / 1920
        widget_width = int(300 * scale_factor)
        colorbar_widget.setFixedWidth(widget_width)

        layout = QtWidgets.QVBoxLayout()
        layout.setAlignment(QtCore.Qt.AlignTop)
        layout.setSpacing(0)
        colorbar_widget.setLayout(layout)

        valid_growth = self.growth_data[self.current_reference]['valid_growth']
        min_growth = min(valid_growth) if len(valid_growth) > 0 else 0.25
        max_growth = max(valid_growth) if len(valid_growth) > 0 else 1.0
        mean_growth = np.mean(valid_growth) if len(valid_growth) > 0 else (min_growth + max_growth) / 2

        img_width = int(100 * scale_factor)
        text_width = int(150 * scale_factor)
        total_img_width = img_width + text_width
        img_height = int(800 * scale_factor)
        title_height = int(50 * scale_factor)
        total_img_height = img_height + title_height
        image = Image.new('RGB', (total_img_width, total_img_height), (255, 255, 255))
        draw = ImageDraw.Draw(image)

        segment_height = img_height // 4
        color_map = {'white': (255, 255, 255), 'gray': (128, 128, 128), 'black': (0, 0, 0)}
        non_growth_rgb = color_map.get(self.non_growth_color.lower(), (255, 255, 255))
        draw.rectangle([0, title_height, img_width, title_height + segment_height], fill=(0, 255, 0))
        draw.rectangle([0, title_height + segment_height, img_width, title_height + segment_height * 2], fill=(0, 0, 255))
        draw.rectangle([0, title_height + segment_height * 2, img_width, title_height + segment_height * 3], fill=(255, 0, 0))
        draw.rectangle([0, title_height + segment_height * 3, img_width, title_height + img_height], fill=non_growth_rgb)

        font_size = max(int(16 * scale_factor), 16)
        title_font_size = max(int(20 * scale_factor), 20)
        try:
            font = ImageFont.truetype("fonts/ARIAL.TTF", font_size)
            title_font = ImageFont.truetype("fonts/ARIAL.TTF", title_font_size)
        except:
            font = ImageFont.load_default()
            title_font = ImageFont.load_default()
            print("Warning: Failed to load Arial font, using default font. Text may be less readable.")

        title_text = "Growth"
        title_bbox = draw.textbbox((0, 0), title_text, font=title_font)
        title_width = title_bbox[2] - title_bbox[0]
        draw.text(((img_width - title_width) // 2, int(10 * scale_factor)), title_text, fill=(0, 0, 0), font=title_font)

        numeric_labels = [
            (f"{mean_growth + self.growth_mu:.2f} to {max_growth:.2f}", title_height + int(segment_height * 0.5)),
            (f"{mean_growth - self.growth_mu:.2f} to {mean_growth + self.growth_mu:.2f}", title_height + int(segment_height * 1.5)),
            (f"{min_growth:.2f} to {mean_growth - self.growth_mu:.2f}", title_height + int(segment_height * 2.5)),
            ("NaN/Non-filtered", title_height + int(segment_height * 3.5))
        ]

        for text, y_pos in numeric_labels:
            text_bbox = draw.textbbox((0, 0), text, font=font)
            text_width = text_bbox[2] - text_bbox[0]
            draw.text(((img_width - text_width) // 2, y_pos - font_size // 2), text, fill=(0, 0, 0), font=font)

        text_labels = [
            ("Above Average", title_height + int(segment_height * 0.5)),
            ("Average", title_height + int(segment_height * 1.5)),
            ("Below Average", title_height + int(segment_height * 2.5)),
            ("No-Growth", title_height + int(segment_height * 3.5))
        ]

        for text, y_pos in text_labels:
            draw.text((img_width + int(15 * scale_factor), y_pos - font_size // 2), text, fill=(0, 0, 0), font=font)

        img_byte_arr = io.BytesIO()
        image.save(img_byte_arr, format='PNG')
        img_byte_arr.seek(0)
        pixmap = QtGui.QPixmap()
        pixmap.loadFromData(img_byte_arr.getvalue())

        colorbar_label = QtWidgets.QLabel()
        colorbar_label.setPixmap(pixmap)
        colorbar_label.setFixedSize(total_img_width, total_img_height)
        layout.addWidget(colorbar_label, alignment=QtCore.Qt.AlignTop)

        layout.addStretch()
        return colorbar_widget

    def update_colorbar(self):
        while self.colorbar_container.layout().count():
            item = self.colorbar_container.layout().takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        colorbar_widget = self.create_colorbar()
        self.colorbar_container.layout().addWidget(colorbar_widget)
        self.colorbar_container.layout().addStretch()

    def select_reference(self, filename):
        self.current_reference = filename
        for button in self.findChildren(QtWidgets.QPushButton):
            button.setChecked(button.text() == os.path.basename(filename))
        self.update_point_clouds()
        self.update_colorbar()
        self.vtk_widget.GetRenderWindow().Render()

    def update_point_clouds(self):
        for actor in self.actors.values():
            self.renderer.RemoveActor(actor)
        self.actors.clear()

        data = self.growth_data[self.current_reference]
        com_points = data['com_points']
        colors = data['colors']

        vtk_points = vtk.vtkPoints()
        for pt in com_points:
            vtk_points.InsertNextPoint(pt[0], pt[1], pt[2])

        vertices = vtk.vtkCellArray()
        for i in range(com_points.shape[0]):
            vertices.InsertNextCell(1)
            vertices.InsertCellPoint(i)

        point_polydata = vtk.vtkPolyData()
        point_polydata.SetPoints(vtk_points)
        point_polydata.SetVerts(vertices)

        vtk_color = vtk.vtkUnsignedCharArray()
        vtk_color.SetNumberOfComponents(3)
        vtk_color.SetName("Colors")
        for color in colors:
            vtk_color.InsertNextTuple3(*(np.array(color) * 255))

        point_polydata.GetPointData().SetScalars(vtk_color)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(point_polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(self.point_size)
        self.actors['combined'] = actor
        self.renderer.AddActor(actor)

def sort_filenames_by_date(filenames):
    def extract_date(filename):
        return datetime.strptime(filename.replace('.pcd', ''), '%d-%m-%y')
    return sorted(filenames, key=extract_date)

def run():
    config = load_yaml_config(growth_yaml_path)
    if not config:
        print("Failed to load configuration. Using default values.")
        config = {
            'pcd_directory': 'maps',
            'cell_size': 0.8,
            'ranges': [2.2, 64, -10, 1, -2, -0.1],
            'growth_mu': 0.2,
            'point_size': 3,
            'gui_background': 'black',
            'non_growth_points_color': 'white'
        }
    else:
        print("Loaded Config")
        print(config)
    pcd_directory = config['pcd_directory']
    filenames = sort_filenames_by_date([f for f in os.listdir(pcd_directory) if f.endswith('.pcd')])
    file_paths = [os.path.join(pcd_directory, f) for f in filenames]
    if not file_paths:
        print(f"No PCD files found in {pcd_directory}")
        return

    app = QtWidgets.QApplication(sys.argv)
    window = PointCloudGrowthApp(
        pointcloud_files=file_paths,
        cell_size=config['cell_size'],
        ranges=config['ranges'],
        point_size=3,
        growth_mu=config['growth_mu'],
        gui_background=config['GUI_background'],
        non_growth_color=config['non_growth_points_color']
    )
    sys.exit(app.exec_())

if __name__ == "__main__":
    run()
