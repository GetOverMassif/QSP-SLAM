import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QFileDialog
import open3d as o3d

class AnnotationTool(QMainWindow):
    def __init__(self, parent=None):
        super(AnnotationTool, self).__init__(parent)

        self.setWindowTitle("3D Annotation Tool")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout(self.central_widget)

        self.open3d_widget = o3d.visualization.draw_geometries_with_editing([], "Edit")

        

        self.layout.addWidget(self.open3d_widget)



        # self.button = QPushButton("Load Data", self)
        # self.layout.addWidget(self.button)

        # self.button.clicked.connect(self.load_data)

    # def load_data(self):
    #     options = QFileDialog.Options()
    #     options |= QFileDialog.ReadOnly
    #     file_path, _ = QFileDialog.getOpenFileName(self, "Open Point Cloud or Mesh File", "", "PointCloud Files (*.pcd);;Mesh Files (*.ply);;All Files (*)", options=options)

    #     if file_path:
    #         # Load your data, for example, load a point cloud from file_path
    #         # Replace the following lines with your actual data loading and visualization code
    #         pcd = o3d.io.read_point_cloud(file_path)
    #         o3d.visualization.draw_geometries_with_editing([pcd])

    #         print(f"Data loaded from file: {file_path}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AnnotationTool()
    window.show()
    sys.exit(app.exec_())