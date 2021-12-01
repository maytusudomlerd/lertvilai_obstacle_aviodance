import open3d.visualization.gui as gui

def guirun():
    app = gui.Application.instance
    app.initialize()

    points = make_point_cloud(100, (0, 0, 0), 1.0)

    vis = o3d.visualization.O3DVisualizer("Open3D - 3D Text", 1024, 768)
    # vis.show_settings = True
    vis.add_geometry("Points", points)
    for idx in range(0, len(points.points)):
        vis.add_3d_label(points.points[idx], "{}".format(idx))
    vis.reset_camera_to_default()

    app.add_window(vis)
    app.run()