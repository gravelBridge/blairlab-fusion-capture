# Fusion 360 Rodent-Eye Capture - Blair Lab

Render 128×128 stereo “rodent-eye” images from specified grid positions in an Autodesk Fusion 360 design. Useful for behavioral or robotics simulation datasets where a rodent-like camera model is needed. Made for Blair Lab's research by John Tian.

What it does
- Reads grid positions (outlined in the Blair Lab maze positions Google Sheet) from positions.txt
- Maps grid to model coordinates via anchor points in the Fusion 360 design (axes swapped)
- For each grid cell, renders 8 images:
  - Left/right eye, each facing four directions
  - Corner cells use diagonal directions
- Saves images to photos/<file_prefix> with deterministic names

Camera model
- Eye separation: 0.5 inches (±0.25" along right vector)
- Eye height: 33.577" + 2.5"
- Eye yaw: ±50° from the base direction (left/right)
- Pitch: +15°
- Vertical FOV: 150°
- Output: 128×128 PNG
- “North” is the direction from (5,5) toward (5,0), for more information see the Blair Lab maze positions Google Sheet.

File structure
- photos/
- capture.py
- positions.txt

positions.txt format
- First line: file prefix (subfolder under photos)
- Following lines: grid coordinates as x,y (integers)

Example
```
config0
5,7
5,8
5,9
```

Output naming
- photos/<file_prefix>/<file_prefix>_<x>_<y>_<direction>_<eye>.png
- Directions at non-corner cells: north, east, south, west
- Directions at corner cells (0,0), (10,0), (0,10), (10,10): NE, SE, SW, NW
- Example: photos/config0/config0_5_5_north_left.png

Setup and run
1. Open your design in Fusion 360 (Design workspace).
2. Place capture.py and positions.txt next to a photos/ folder (script will create photos/ if missing).
3. Tools → Scripts and Add-Ins → Scripts → Add… → select capture.py → Run.
4. The script switches to the Render workspace and performs local renders. Images are overwritten if they exist.

Notes
- Coordinate mapping uses provided anchors with linear interpolation:
  - model X from grid y; model Y from grid x; Z = 33.577"
- Requires the Render workspace and local rendering capability:
  - In Fusion 360 UI, the Render workspace must be available (FusionRenderEnvironment).
- If you see errors about rendering or no active design, ensure a design document is open and active before running.
