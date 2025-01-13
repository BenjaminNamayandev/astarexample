#!/usr/bin/env python3
"""
debug_hardcoded.py

A debugging script that:
1) Hard-codes the DXF file name: "FNB1.dxf".
2) Lists layouts in the DXF.
3) Prints entity types in the model space.
4) If an entity is an INSERT (block reference), shows what's in that block.
5) Gathers LINE/LWPOLYLINE/ARC in model space and tries a bounding box.

Usage:
    python debug_hardcoded.py
(No arguments needed; "FNB1.dxf" must be in the same folder as this script.)
"""

import sys
import ezdxf

def main():
    # Hard-coded DXF filename
    dxf_file = "FNB1.dxf"

    print("DEBUG SCRIPT STARTED!")
    print(f"Using DXF file: {dxf_file}")

    # 1) Attempt to read the DXF
    try:
        doc = ezdxf.readfile(dxf_file)
    except Exception as ex:
        print("Exception while reading DXF:", ex)
        sys.exit(1)

    # 2) List all layouts
    print("\n--- Layouts in this DXF ---")
    for layout in doc.layouts:  # changed from doc.layout_names
        print(f"  - {layout.name}")

    # 3) Print entity types in model space
    msp = doc.modelspace()
    all_msp_entities = list(msp.query("*"))  # everything in model space
    print(f"\nFound {len(all_msp_entities)} entities in MODEL space.\n")

    # We'll track lines, polylines, arcs for bounding box
    candidate_entities = []

    for i, e in enumerate(all_msp_entities, start=1):
        dxftype = e.dxftype()
        layer = e.dxf.layer
        print(f"{i}: dxftype={dxftype}, layer={layer}")

        # If it's an INSERT, let's see what's in that block
        if dxftype == "INSERT":
            block_name = e.dxf.name
            print(f"   -> This is a block reference to '{block_name}'. Listing block contents:")
            if block_name in doc.blocks:
                block_def = doc.blocks[block_name]
                for j, be in enumerate(block_def, start=1):
                    print(f"     Block entity {j}: dxftype={be.dxftype()}, layer={be.dxf.layer}")
            else:
                print(f"   -> WARNING: Block '{block_name}' not found in doc.blocks")

        # Keep track if it's a line/lwpolyline/arc for bounding box
        if dxftype in ("LINE", "LWPOLYLINE", "ARC"):
            candidate_entities.append(e)

    # 4) Attempt a simple bounding box on (LINE, LWPOLYLINE, ARC)
    min_x = float('inf')
    min_y = float('inf')
    max_x = float('-inf')
    max_y = float('-inf')

    for e in candidate_entities:
        # Some entities may not have .bbox()
        if hasattr(e, "bbox"):
            bbox = e.bbox()  # (xmin, ymin, xmax, ymax)
            if bbox:
                x0, y0, x1, y1 = bbox
                min_x = min(min_x, x0)
                min_y = min(min_y, y0)
                max_x = max(max_x, x1)
                max_y = max(max_y, y1)

    print("\n--- Bounding Box from (LINE, LWPOLYLINE, ARC) in Model Space ---")
    if min_x == float('inf') and max_x == float('-inf'):
        print("No line/polyline/arc geometry found in model space.")
    else:
        print(f"x range: {min_x} to {max_x}")
        print(f"y range: {min_y} to {max_y}")

    print("\nDone debugging.")

if __name__ == "__main__":
    main()
