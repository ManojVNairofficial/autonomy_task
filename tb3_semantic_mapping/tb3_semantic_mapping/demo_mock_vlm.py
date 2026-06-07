#!/usr/bin/env python3
"""Standalone demo of mocked image-to-label tagging (no ROS / Gazebo needed).

Simulated or mocked examples of image-to-label tagging (e.g. 'this image contains a toilet')."

It feeds real images into the same `MockVLM` the live tagger node uses and prints the label + human-readable description for each, exactly as the robot would log them during exploration. 

Run it with:
    ros2 run tb3_semantic_mapping demo_vlm
    # or directly:
    python3 demo_mock_vlm.py

"""

from __future__ import annotations

import os

import numpy as np

from tb3_semantic_mapping.vlm_interface import make_vlm


def _load_sign_images():
    """Return [(name, rgb_image), ...] for the room-sign PNGs."""
    import cv2
    try:
        from ament_index_python.packages import get_package_share_directory
        signs_dir = os.path.join(
            get_package_share_directory('tb3_semantic_mapping'), 'signs')
    except Exception:
        # fall back to the source tree when run before install
        signs_dir = os.path.join(os.path.dirname(__file__), '..', 'signs')

    out = []
    for label in ('toilet', 'kitchen', 'office'):
        path = os.path.join(signs_dir, f'{label}_sign.png')
        bgr = cv2.imread(path)
        if bgr is not None:
            out.append((f'{label}_sign.png', cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
    return out


def main():
    vlm = make_vlm('mock')

    print('=' * 64)
    print(' Mocked image-to-label tagging  (MockVLM — the simulation backend)')
    print('=' * 64)

    samples = _load_sign_images()
    # A blank grey frame stands in for "camera sees an empty hallway".
    samples.append(('blank_frame (empty corridor)',
                    np.full((240, 320, 3), 128, dtype=np.uint8)))

    if len(samples) <= 1:
        print('\n[!] Sign images not found. Build & source the package first:')
        print('    colcon build --packages-select tb3_semantic_mapping')
        print('    source install/setup.bash')

    for name, rgb in samples:
        r = vlm.classify(rgb)
        print(f'\n  image: {name}')
        print(f'    -> label       : {r.label}')
        print(f'    -> confidence  : {r.confidence:.2f}')
        print(f'    -> description : "{r.description}"')

    print('\n' + '=' * 64)
    print(' A real VLM (CLIP / GPT-4o) is a drop-in replacement: same call,')
    print(' same VLMResult — switch with make_vlm("clip"). See README §5.')
    print('=' * 64)


if __name__ == '__main__':
    main()
