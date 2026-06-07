"""General text-query -> room-label matching

Matching is two-stage and purely data-driven:
  1. exact synonym hit   (e.g. "restroom" -> toilet)            score += 1.0
  2. fuzzy hit (difflib) (e.g. "toilett", "kitchin", plurals)  score += 0.6
The label with the highest score wins; an unrelated query ("gym") scores 0 and
returns None, so the caller can answer "not found" .
"""

from __future__ import annotations

import difflib
import re


# Natural words -> canonical room label. Adding a room is a data change only.
SYNONYMS = {
    'kitchen': ['kitchen', 'pantry', 'canteen', 'cafeteria', 'break', 'breakroom',
                'coffee', 'food', 'eat', 'dining', 'lunch'],
    'toilet':  ['toilet', 'restroom', 'bathroom', 'washroom', 'wc', 'loo',
                'lavatory', 'urinal', 'potty'],
    'office':  ['office', 'desk', 'desks', 'workplace', 'workspace', 'work',
                'cubicle', 'workstation', 'meeting', 'room'],
    'hallway': ['hallway', 'corridor', 'hall', 'passage', 'aisle', 'walkway',
                'lobby', 'entrance'],
}

_FUZZY_CUTOFF = 0.8


def resolve_query(query: str, known_labels):
    """Return (label, score) for the best-matching known room, or (None, 0.0).

    `known_labels` is the set of rooms actually present in the semantic map, so
    we never return a place the robot has not seen.
    """
    tokens = re.findall(r'[a-z]+', query.lower())
    if not tokens:
        return None, 0.0

    best_label, best_score = None, 0.0
    for label in known_labels:
        vocab = set(SYNONYMS.get(label, [label]))
        score = 0.0
        for tok in tokens:
            if tok in vocab:
                score += 1.0                                  # exact synonym
            elif difflib.get_close_matches(tok, vocab, n=1, cutoff=_FUZZY_CUTOFF):
                score += 0.6                                  # fuzzy / typo / plural
        if score > best_score:
            best_label, best_score = label, score

    if best_label is None or best_score == 0.0:
        return None, 0.0
    return best_label, best_score
