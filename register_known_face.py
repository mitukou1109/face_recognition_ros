import os
import shutil
import sys

from ament_index_python.packages import get_package_share_directory

if len(sys.argv) < 2:
    print("usage: python3 register_known_face.py <known_face_image> [name]")
    sys.exit(1)

if not os.path.isfile(sys.argv[1]):
    print("Invalid path")
    sys.exit(1)

KNOWN_FACE_DIR = os.path.join(
    get_package_share_directory("face_recognition_ros"),
    "resource",
    "known_faces",
)
os.makedirs(KNOWN_FACE_DIR, exist_ok=True)

file_name, file_ext = os.path.splitext(sys.argv[1])
face_name = sys.argv[2] if len(sys.argv) >= 3 else file_name
shutil.copyfile(
    sys.argv[1],
    os.path.join(KNOWN_FACE_DIR, f"{face_name}{file_ext}"),
)

print(f"Registered {face_name}")
