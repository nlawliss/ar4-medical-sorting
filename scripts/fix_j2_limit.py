import os, sys
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

"""Update J2 positive limit in ARbot_safe.cal"""
import pickle, os

CAL_PATH = os.path.join(REPO_ROOT, "config", "ARbot_safe.cal")

cal = pickle.load(open(CAL_PATH, "rb"))

# J2 positive limit is at index 135
print(f"J2 PosLim before: {cal[135]}")
cal[135] = "51.0"
print(f"J2 PosLim after:  {cal[135]}")

pickle.dump(cal, open(CAL_PATH, "wb"))
print("Saved.")
