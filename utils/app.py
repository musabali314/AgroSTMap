import sys
import random
import vtkmodules.all as vtk
import open3d as o3d
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import os

def generate_random_colors(n):
    return [[random.random(), random.random(), random.random()] for _ in range(n)]


class PointCloudApp(QtWidgets.QMainWindow):
    def __init__(self, pointcloud_files, colors=None, point_size=3):
        super().__init__()
        self.setWindowTitle("Multi PointCloud Viewer")

        # Set window icon (logo)
        try:
            icon_path = "logo.png"  # Replace with your icon file path (e.g., logo.png, logo.ico)
            self.setWindowIcon(QtGui.QIcon(icon_path))
            print(f"Window icon set to: {icon_path}")
        except Exception as e:
            print(f"Failed to set window icon: {e}")

        if colors is None or len(colors) != len(pointcloud_files):
            colors = generate_random_colors(len(pointcloud_files))

        self.actors = {}
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.1)

        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)

        # Set up custom interactor style with adjusted mouse sensitivity
        interactor_style = vtk.vtkInteractorStyleTrackballCamera()
        interactor_style.SetMotionFactor(5.5)  # Adjusted for responsiveness
        self.vtk_widget.SetInteractorStyle(interactor_style)
        print("Interactor style set to:", interactor_style.__class__.__name__)  # Debug print

        sidebar_layout = QtWidgets.QVBoxLayout()

        for idx, (file, color) in enumerate(zip(pointcloud_files, colors)):
            pcd = o3d.io.read_point_cloud(file)
            pcd.paint_uniform_color(color)
            points = np.asarray(pcd.points)
            print(f"Point cloud {file} bounds: min={points.min(axis=0)}, max={points.max(axis=0)}")  # Debug bounds

            vtk_points = vtk.vtkPoints()
            for pt in points:
                vtk_points.InsertNextPoint(pt[0], pt[1], pt[2])

            vertices = vtk.vtkCellArray()
            for i in range(points.shape[0]):
                vertices.InsertNextCell(1)
                vertices.InsertCellPoint(i)

            point_polydata = vtk.vtkPolyData()
            point_polydata.SetPoints(vtk_points)
            point_polydata.SetVerts(vertices)

            vtk_color = vtk.vtkUnsignedCharArray()
            vtk_color.SetNumberOfComponents(3)
            vtk_color.SetName("Colors")
            for _ in range(points.shape[0]):
                vtk_color.InsertNextTuple3(*(np.array(color) * 255))
            point_polydata.GetPointData().SetScalars(vtk_color)

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(point_polydata)

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetPointSize(point_size)  # Set point size for the actor
            self.actors[file] = actor
            self.renderer.AddActor(actor)

            # Create checkbox with colored label
            #checkbox = QtWidgets.QCheckBox(file)
            checkbox = QtWidgets.QCheckBox(os.path.basename(file))
            checkbox.setChecked(True)
            checkbox.stateChanged.connect(lambda state, f=file: self.toggle_pointcloud(f, state))

            color_label = QtWidgets.QLabel()
            pixmap = QtGui.QPixmap(20, 20)
            r, g, b = (np.array(color) * 255).astype(int)
            pixmap.fill(QtGui.QColor(r, g, b))
            color_label.setPixmap(pixmap)

            hbox = QtWidgets.QHBoxLayout()
            hbox.addWidget(color_label)
            hbox.addWidget(checkbox)

            container = QtWidgets.QWidget()
            container.setLayout(hbox)
            sidebar_layout.addWidget(container)

        sidebar_layout.addStretch()
        sidebar_widget = QtWidgets.QWidget()
        sidebar_widget.setLayout(sidebar_layout)

        splitter = QtWidgets.QSplitter()
        splitter.addWidget(sidebar_widget)
        splitter.addWidget(self.vtk_widget)
        splitter.setStretchFactor(1, 1)

        self.setCentralWidget(splitter)
        self.show()
        self.vtk_widget.Initialize()
        self.vtk_widget.Start()

        # Reset camera to ensure proper view
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_pointcloud(self, filename, state):
        actor = self.actors.get(filename)
        if actor:
            if state == QtCore.Qt.Checked:
                self.renderer.AddActor(actor)
            else:
                self.renderer.RemoveActor(actor)
            self.vtk_widget.GetRenderWindow().Render()


def run(pointcloud_files, colors=None, point_size=2):
    app = QtWidgets.QApplication(sys.argv)
    window = PointCloudApp(pointcloud_files, colors, point_size)
    sys.exit(app.exec_())
