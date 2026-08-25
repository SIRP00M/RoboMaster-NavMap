import math
import numpy as np
import cv2

def normalize_angle_deg(a):
    return (a % 360.0 + 360.0) % 360.0

def line_intersection(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(den) < 1e-6:
        return None
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
    ix = x1 + t*(x2-x1)
    iy = y1 + t*(y2-y1)
    return (ix, iy)

def point_to_segment_dist(p, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    l2 = dx*dx + dy*dy
    if l2 == 0:
        return math.hypot(p[0]-a[0], p[1]-a[1])
    t = max(0, min(1, ((p[0]-a[0])*dx + (p[1]-a[1])*dy) / l2))
    proj = (a[0] + t*dx, a[1] + t*dy)
    return math.hypot(p[0]-proj[0], p[1]-proj[1])

def get_dist(cm_str, max_range_m=0.45):
    if cm_str is None or str(cm_str).strip() == "": return None
    d = float(cm_str)
    if 5.0 <= d <= max_range_m * 100.0:
        return d / 100.0
    return None

def extract_segments(samples):
    segments = []
    current_segment = {"type": "straight", "indices": []}
    
    for i, s in enumerate(samples):
        if i == 0:
            current_segment["indices"].append(i)
            continue
            
        dy = abs(s["yaw"] - samples[i-1]["yaw"])
        delta_yaw = dy if dy <= 180 else 360 - dy
        
        is_turning = delta_yaw >= 3.0
        
        if is_turning:
            if current_segment["type"] == "straight":
                if len(current_segment["indices"]) > 0:
                    segments.append(current_segment)
                current_segment = {"type": "turn", "indices": [i]}
            else:
                current_segment["indices"].append(i)
        else:
            if current_segment["type"] == "turn":
                if len(current_segment["indices"]) > 0:
                    segments.append(current_segment)
                current_segment = {"type": "straight", "indices": [i]}
            else:
                current_segment["indices"].append(i)
                
    if len(current_segment["indices"]) > 0:
        segments.append(current_segment)
    return segments

def process_map_data(samples):
    raw_s = []
    for s in samples:
        raw_s.append({
            "x": float(s.get("map_x", s.get("pose_x", 0))),
            "y": float(s.get("map_y", s.get("pose_y", 0))),
            "yaw": float(s.get("theta_deg", s.get("yaw_deg", 0))),
            "left_cm": s.get("left_cm"),
            "right_cm": s.get("right_cm")
        })
        
    if not raw_s: return [], [], []
    
    # 1. Global Auto-Align to Cardinal
    #    Use BIDIRECTIONAL heading (mod 180) so that 117° and -63° (=297°) 
    #    are both recognized as the SAME corridor axis (~117° mod 180 = 117°, 
    #    297° mod 180 = 117°).
    initial_segments = extract_segments(raw_s)
    straight_segs = []
    for seg in initial_segments:
        if seg["type"] == "straight" and len(seg["indices"]) >= 3:
            yaws = [raw_s[idx]["yaw"] for idx in seg["indices"]]
            sin_sum = sum(math.sin(math.radians(y)) for y in yaws)
            cos_sum = sum(math.cos(math.radians(y)) for y in yaws)
            avg_yaw = math.degrees(math.atan2(sin_sum, cos_sum))
            
            p0 = raw_s[seg["indices"][0]]
            pe = raw_s[seg["indices"][-1]]
            length = math.hypot(pe["x"]-p0["x"], pe["y"]-p0["y"])
            straight_segs.append({"avg_yaw": avg_yaw, "length": length})
            
    rot_angle_deg = 0.0
    if straight_segs:
        longest_seg = max(straight_segs, key=lambda s: s["length"])
        # Use bidirectional: reduce to 0-180 range, then snap to nearest
        # cardinal axis (0, 90, 180)
        longest_yaw = normalize_angle_deg(longest_seg["avg_yaw"])
        bidir_yaw = longest_yaw % 180.0  # 117° → 117°, 297° → 117°
        nearest_cardinal = round(bidir_yaw / 90.0) * 90.0  # 117° → 90°
        rot_angle_deg = nearest_cardinal - bidir_yaw  # 90° - 117° = -27°
        
        # But we need to apply this relative to the original full angle
        # The rotation should align this axis to the nearest cardinal
        rot_angle_deg = nearest_cardinal - bidir_yaw

    rotated_samples = []
    start_x, start_y = raw_s[0]["x"], raw_s[0]["y"]
    rad = math.radians(rot_angle_deg)
    cos_r = math.cos(rad)
    sin_r = math.sin(rad)
    
    for s in raw_s:
        dx = s["x"] - start_x
        dy = s["y"] - start_y
        rx = dx*cos_r - dy*sin_r
        ry = dx*sin_r + dy*cos_r
        ryaw = normalize_angle_deg(s["yaw"] + rot_angle_deg)
        
        new_s = s.copy()
        new_s["x"] = rx
        new_s["y"] = ry
        new_s["yaw"] = ryaw
        rotated_samples.append(new_s)
        
    def to_plot_coords(x, y): return y, x

    # 2. Extract final segments
    segments = extract_segments(rotated_samples)
    
    segment_geoms = []
    for seg in segments:
        indices = seg["indices"]
        if seg["type"] == "turn" or len(indices) < 3:
            continue
            
        yaws = [rotated_samples[idx]["yaw"] for idx in indices]
        sin_sum = sum(math.sin(math.radians(y)) for y in yaws)
        cos_sum = sum(math.cos(math.radians(y)) for y in yaws)
        avg_yaw = math.degrees(math.atan2(sin_sum, cos_sum))
        
        rad = math.radians(avg_yaw)
        u_x, u_y = math.cos(rad), math.sin(rad)
        
        p0_x = rotated_samples[indices[0]]["x"]
        p0_y = rotated_samples[indices[0]]["y"]
        
        proj_pts = []
        for idx in indices:
            s = rotated_samples[idx]
            dx = s["x"] - p0_x
            dy = s["y"] - p0_y
            d = dx*u_x + dy*u_y
            proj_pts.append((d, idx))
            
        min_d = min(p[0] for p in proj_pts)
        max_d = max(p[0] for p in proj_pts)
        
        lefts, rights = [], []
        for idx in indices:
            s = rotated_samples[idx]
            ld = get_dist(s["left_cm"])
            rd = get_dist(s["right_cm"])
            if ld: lefts.append(ld)
            if rd: rights.append(rd)
            
        avg_l = sorted(lefts)[len(lefts)//2] if lefts else 0.22
        avg_r = sorted(rights)[len(rights)//2] if rights else 0.22
        
        U_px, U_py = to_plot_coords(u_x, u_y)
        N_px, N_py = -U_py, U_px
        P0_px, P0_py = to_plot_coords(p0_x, p0_y)
        
        C_start = (P0_px + min_d*U_px, P0_py + min_d*U_py)
        C_end   = (P0_px + max_d*U_px, P0_py + max_d*U_py)
        
        L_start = (C_start[0] + avg_l*N_px, C_start[1] + avg_l*N_py)
        L_end   = (C_end[0]   + avg_l*N_px, C_end[1]   + avg_l*N_py)
        
        R_start = (C_start[0] - avg_r*N_px, C_start[1] - avg_r*N_py)
        R_end   = (C_end[0]   - avg_r*N_px, C_end[1]   - avg_r*N_py)
        
        segment_geoms.append({
            "indices": indices, "has_left": bool(lefts), "has_right": bool(rights),
            "L_start": L_start, "L_end": L_end,
            "R_start": R_start, "R_end": R_end,
            "C_start": C_start, "C_end": C_end,
            "avg_yaw": avg_yaw
        })
        
    # 3. Corner Intersection Snapping (Miter Filling)
    for i in range(len(segment_geoms) - 1):
        g1 = segment_geoms[i]
        for j in range(i+1, min(i+3, len(segment_geoms))):
            g2 = segment_geoms[j]
            
            # ONLY intersect if they form a corner (angle difference > 30 and < 150)
            diff = abs(normalize_angle_deg(g1["avg_yaw"] - g2["avg_yaw"]))
            if diff > 180: diff = 360 - diff
            if diff < 30 or diff > 150:
                continue # Parallel or anti-parallel, skip intersection
                
            # Check distance to prevent teleporting intersections
            end_to_start_dist = math.hypot(
                g1["C_end"][0]-g2["C_start"][0], 
                g1["C_end"][1]-g2["C_start"][1]
            )
            if end_to_start_dist > 1.0:
                continue
                
            # Intersect Left Wall with safety clamp
            L_int = line_intersection(g1["L_start"], g1["L_end"], g2["L_start"], g2["L_end"])
            if L_int:
                # Safety: don't let intersection be too far from midpoint
                mid = ((g1["C_end"][0]+g2["C_start"][0])/2, (g1["C_end"][1]+g2["C_start"][1])/2)
                if math.hypot(L_int[0]-mid[0], L_int[1]-mid[1]) < 1.5:
                    g1["L_end"] = L_int
                    g2["L_start"] = L_int
                
            # Intersect Right Wall with safety clamp
            R_int = line_intersection(g1["R_start"], g1["R_end"], g2["R_start"], g2["R_end"])
            if R_int:
                mid = ((g1["C_end"][0]+g2["C_start"][0])/2, (g1["C_end"][1]+g2["C_start"][1])/2)
                if math.hypot(R_int[0]-mid[0], R_int[1]-mid[1]) < 1.5:
                    g1["R_end"] = R_int
                    g2["R_start"] = R_int
                
            # Intersect Centerline with safety clamp
            C_int = line_intersection(g1["C_start"], g1["C_end"], g2["C_start"], g2["C_end"])
            if C_int:
                mid = ((g1["C_end"][0]+g2["C_start"][0])/2, (g1["C_end"][1]+g2["C_start"][1])/2)
                if math.hypot(C_int[0]-mid[0], C_int[1]-mid[1]) < 1.5:
                    g1["C_end"] = C_int
                    g2["C_start"] = C_int
            break

    # 4. Generate Drawing Primitives (Polygons and Trajectory)
    polygons = []
    trajectory_pts = []
    
    for idx, g in enumerate(segment_geoms):
        poly = [g["L_start"], g["L_end"], g["R_end"], g["R_start"]]
        polygons.append(poly)
        
        trajectory_pts.append(g["C_start"])
        trajectory_pts.append(g["C_end"])
        
    # 4b. Miter Corner Fill Polygons (fill the gap between adjacent corridor quads)
    for i in range(len(segment_geoms) - 1):
        g1 = segment_geoms[i]
        g2_candidates = segment_geoms[i+1:i+3]
        for g2 in g2_candidates:
            end_to_start = math.hypot(
                g1["C_end"][0]-g2["C_start"][0], 
                g1["C_end"][1]-g2["C_start"][1]
            )
            if end_to_start < 0.8:
                # Fill corner polygon
                corner_poly = [g1["L_end"], g2["L_start"], g2["R_start"], g1["R_end"]]
                polygons.append(corner_poly)
                break
        
    # 5. Extract Continuous Wall Lines using OpenCV Contour
    wall_lines = []
    if polygons:
        # Determine bounds
        all_pts = [p for poly in polygons for p in poly]
        min_x, max_x = min(p[0] for p in all_pts), max(p[0] for p in all_pts)
        min_y, max_y = min(p[1] for p in all_pts), max(p[1] for p in all_pts)
        
        # Add padding (1 meter)
        pad = 1.0
        min_x -= pad; max_x += pad
        min_y -= pad; max_y += pad
        
        # Resolution: 100 pixels per meter (1cm per pixel)
        resolution = 100
        width = int((max_x - min_x) * resolution)
        height = int((max_y - min_y) * resolution)
        
        if width > 0 and height > 0:
            img = np.zeros((height, width), dtype=np.uint8)
            
            # Convert polygons to pixel coordinates
            pixel_polys = []
            for poly in polygons:
                px_poly = []
                for x, y in poly:
                    px = int((x - min_x) * resolution)
                    py = int((y - min_y) * resolution)
                    px_poly.append([px, py])
                # Use convex hull to prevent self-intersecting bowties at corners
                px_arr = np.array(px_poly, dtype=np.int32)
                hull = cv2.convexHull(px_arr)
                pixel_polys.append(hull)
                
            # Draw solid white polygons
            cv2.fillPoly(img, pixel_polys, 255)
            
            # Apply Morphological Close to fuse sliver gaps between disjoint segment polygons
            kernel = np.ones((25, 25), np.uint8)
            img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
            
            # Extract contours
            contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            # Convert contours back to line segments in map coordinates
            for cnt in contours:
                # Filter out tiny micro-cavities (area < 500 square pixels, i.e., 0.05 m^2)
                if cv2.contourArea(cnt) < 500:
                    continue
                    
                # Simplify contour to remove pixel staircase effect
                epsilon = 2.0
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                if len(approx) < 2:
                    continue
                approx = approx.squeeze(axis=1) # shape (N, 2)
                if len(approx.shape) == 1:
                    continue
                    
                for i in range(len(approx)):
                    p1 = approx[i]
                    p2 = approx[(i+1) % len(approx)]
                    
                    x1 = p1[0] / resolution + min_x
                    y1 = p1[1] / resolution + min_y
                    x2 = p2[0] / resolution + min_x
                    y2 = p2[1] / resolution + min_y
                    
                    wall_lines.append([(x1, y1), (x2, y2)])
            
    return polygons, wall_lines, trajectory_pts
