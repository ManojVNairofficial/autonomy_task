"""Automated test cases for the Section 2 semantic system.

Covers the two halves end-to-end without needing Gazebo:
  * image -> label   (MockVLM classifies the room signs correctly)
  * query -> place   (free-text prompt resolves to the right room, incl. example "Where is the toilet?")

Run with:  colcon test --packages-select tb3_semantic_mapping
       or:  pytest src/tb3_semantic_mapping/test/test_semantic.py
"""

import os

import numpy as np
import pytest

from tb3_semantic_mapping.semantic_matching import resolve_query
from tb3_semantic_mapping.vlm_interface import make_vlm

SIGNS_DIR = os.path.join(os.path.dirname(__file__), '..', 'signs')
KNOWN = ['kitchen', 'toilet', 'office', 'hallway']


def _sign(label):
    import cv2
    bgr = cv2.imread(os.path.join(SIGNS_DIR, f'{label}_sign.png'))
    assert bgr is not None, f'missing sign image for {label}'
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


 # ---------------------------------------------------------------------------- #
 #                                image -> label                                #
 # ---------------------------------------------------------------------------- #
@pytest.mark.parametrize('label', ['kitchen', 'toilet', 'office'])
def test_mock_vlm_reads_each_sign(label):
    vlm = make_vlm('mock', signs_dir=SIGNS_DIR)
    result = vlm.classify(_sign(label))
    assert result.label == label
    assert result.confidence >= 0.65
    assert label in result.description.lower()


def test_mock_vlm_empty_frame_is_hallway():
    vlm = make_vlm('mock', signs_dir=SIGNS_DIR)
    blank = np.full((240, 320, 3), 128, dtype=np.uint8)
    assert vlm.classify(blank).label == 'hallway'


# ---------------------------------------------------------------------------- #
#                                query -> place                                #
# ---------------------------------------------------------------------------- #
@pytest.mark.parametrize('query,expected', [
    ('Where is the toilet?', 'toilet'),     
    ('I need the bathroom', 'toilet'),
    ('take me to the restroom', 'toilet'),
    ('where can I make coffee', 'kitchen'),
    ('find the pantry', 'kitchen'),
    ('go to the meeting room', 'office'),
    ('where are the desks', 'office'),
])
def test_query_resolves_to_correct_room(query, expected):
    label, score = resolve_query(query, KNOWN)
    assert label == expected
    assert score > 0.0


def test_query_handles_typos_via_fuzzy_match():
    # "toilett" / "kitchin" are not in any synonym list verbatim.
    assert resolve_query('wheres the toilett', KNOWN)[0] == 'toilet'
    assert resolve_query('the kitchin please', KNOWN)[0] == 'kitchen'


def test_unknown_query_returns_none():
    label, score = resolve_query('where is the gym', KNOWN)
    assert label is None
    assert score == 0.0


def test_query_only_returns_known_places():
    # If the robot has only seen the toilet, a kitchen query must not match it.
    label, _ = resolve_query('where is the kitchen', ['toilet'])
    assert label is None
