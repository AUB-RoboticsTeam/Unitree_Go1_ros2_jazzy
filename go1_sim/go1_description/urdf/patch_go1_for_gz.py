import re, sys, pathlib

p = pathlib.Path("go1.urdf")
txt = p.read_text()

# 1) Comment out Gazebo Classic plugins (keep sensors/links/joints)
classic_plugins = [
    r'libgazebo_ros_control\.so',
    r'libLinkPlot3DPlugin\.so',
    r'libgazebo_ros_openni_kinect\.so',
    r'libgazebo_ros_imu_sensor\.so',
    r'libgazebo_ros_laser\.so',
    r'libunitreeFootContactPlugin\.so',
    r'libunitreeDrawForcePlugin\.so',
]
# Wrap any <gazebo>...</gazebo> block that contains one of those filenames
def comment_plugin_blocks(text):
    pattern = re.compile(
        r'<gazebo\b[^>]*>.*?(?:' + "|".join(classic_plugins) + r').*?</gazebo>',
        flags=re.DOTALL
    )
    def repl(m):
        block = m.group(0)
        return f'<!-- GZ_CLASSIC_OFF\n{block}\nGZ_CLASSIC_OFF -->'
    return re.sub(pattern, repl, text)
txt = comment_plugin_blocks(txt)

# 2) Remove plugin tags nested inside <sensor> blocks (e.g., IMU/cameras/laser plugins)
txt = re.sub(r'<plugin\b[^>]*>.*?</plugin>', '<!-- GZ_CLASSIC_OFF plugin removed -->',
             txt, flags=re.DOTALL)

# 3) Switch lidar type from Classic 'ray' to Gazebo Sim 'gpu_lidar'
txt = re.sub(r'(<sensor[^>]*name="laser"[^>]*type=")ray(")', r'\1gpu_lidar\2', txt)

# Write patched file
out = pathlib.Path("go1_gz.urdf")
out.write_text(txt)
print("Wrote", out)

