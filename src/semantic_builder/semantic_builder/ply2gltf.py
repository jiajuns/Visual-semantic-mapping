#!/usr/bin/env python3
import sys
import struct
import json
import numpy as np
import open3d as o3d
import argparse
import os

def write_glb(points, colors, output_path):
    """
    Export point cloud to GLB (Binary GLTF) format.
    
    Args:
        points: Nx3 numpy array of float32 (positions)
        colors: Nx3 numpy array of float32 (colors, 0.0-1.0) or uint8 (0-255)
        output_path: Path to save the .glb file
    """
    
    # Ensure points are float32
    points = points.astype(np.float32)
    
    # Ensure colors are normalized float32 or convert them
    # GLTF typically expects float vec3 for colors in 0-1 range or uint8 vec3/vec4
    # We will use float32 vec3 0-1 for simplicity as it matches standard GLTF well
    if colors is not None:
        if colors.dtype == np.uint8:
            colors = colors.astype(np.float32) / 255.0
        else:
            colors = colors.astype(np.float32)
            # Clip to 0-1 just in case
            colors = np.clip(colors, 0.0, 1.0)
    
    # --- Prepare Binary Data ---
    
    # 1. Points Buffer
    points_bytes = points.tobytes()
    points_byte_length = len(points_bytes)
    
    # 2. Colors Buffer (if exists)
    colors_bytes = b''
    colors_byte_length = 0
    if colors is not None:
        colors_bytes = colors.tobytes()
        colors_byte_length = len(colors_bytes)
        
    # Combined binary buffer
    # Padding to 4-byte boundary is good practice (float32 is 4 bytes, so usually aligned)
    buffer_data = points_bytes + colors_bytes
    buffer_byte_length = len(buffer_data)
    
    # --- Prepare JSON Header ---
    
    # Accessors
    # 0: POSITION
    accessors = [
        {
            "bufferView": 0,
            "byteOffset": 0,
            "componentType": 5126, # FLOAT
            "count": len(points),
            "type": "VEC3",
            "max": points.max(axis=0).tolist(),
            "min": points.min(axis=0).tolist()
        }
    ]
    
    # 1: COLOR_0 (Optional)
    if colors is not None:
        accessors.append({
            "bufferView": 1,
            "byteOffset": 0,
            "componentType": 5126, # FLOAT
            "count": len(colors),
            "type": "VEC3",
            # No min/max needed for attributes usually, but good for bounding
        })
    
    # BufferViews
    bufferViews = [
        {
            "buffer": 0,
            "byteOffset": 0,
            "byteLength": points_byte_length,
            "target": 34962 # ARRAY_BUFFER
        }
    ]
    
    if colors is not None:
        bufferViews.append({
            "buffer": 0,
            "byteOffset": points_byte_length,
            "byteLength": colors_byte_length,
            "target": 34962 # ARRAY_BUFFER
        })
        
    # Meshes
    attributes = {"POSITION": 0}
    if colors is not None:
        attributes["COLOR_0"] = 1
        
    meshes = [
        {
            "primitives": [
                {
                    "attributes": attributes,
                    "mode": 0 # POINTS
                }
            ]
        }
    ]
    
    # Nodes
    nodes = [
        {
            "mesh": 0
        }
    ]
    
    # Scene
    scene = 0
    scenes = [
        {
            "nodes": [0]
        }
    ]
    
    # Buffers
    buffers = [
        {
            "byteLength": buffer_byte_length
        }
    ]
    
    # Root
    gltf = {
        "asset": {"version": "2.0", "generator": "ply2glb_custom_script"},
        "scene": scene,
        "scenes": scenes,
        "nodes": nodes,
        "meshes": meshes,
        "buffers": buffers,
        "bufferViews": bufferViews,
        "accessors": accessors
    }
    
    json_str = json.dumps(gltf)
    json_bytes = json_str.encode('utf-8')
    
    # Padding JSON to 4-byte boundary with spaces
    padding_len = (4 - (len(json_bytes) % 4)) % 4
    json_bytes += b' ' * padding_len
    
    # Padding Binary to 4-byte boundary with zeros
    # (Our float32 data is likely already multiple of 4, but let's be safe)
    bin_padding_len = (4 - (len(buffer_data) % 4)) % 4
    buffer_data += b'\x00' * bin_padding_len
    
    # GLB Header
    # magic: 0x46546C67 ('glTF')
    # version: 2
    # length: total file size
    
    json_chunk_type = 0x4E4F534A # 'JSON'
    bin_chunk_type = 0x004E4942 # 'BIN\0'
    
    total_length = 12 + 8 + len(json_bytes) + 8 + len(buffer_data)
    
    header = struct.pack('<I', 0x46546C67) + struct.pack('<I', 2) + struct.pack('<I', total_length)
    
    json_chunk_header = struct.pack('<I', len(json_bytes)) + struct.pack('<I', json_chunk_type)
    bin_chunk_header = struct.pack('<I', len(buffer_data)) + struct.pack('<I', bin_chunk_type)
    
    with open(output_path, 'wb') as f:
        f.write(header)
        f.write(json_chunk_header)
        f.write(json_bytes)
        f.write(bin_chunk_header)
        f.write(buffer_data)
        
    print(f"Successfully wrote {output_path} ({len(points)} points)")

def main():
    parser = argparse.ArgumentParser(description="Convert PLY point cloud to GLB (glTF 2.0 binary).")
    parser.add_argument("input", help="Input PLY file path")
    parser.add_argument("output", nargs='?', help="Output GLB file path (optional, defaults to input filename .glb)")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"Error: Input file '{args.input}' not found.")
        sys.exit(1)
        
    output_path = args.output
    if output_path is None:
        base, _ = os.path.splitext(args.input)
        output_path = base + ".glb"
        
    print(f"Reading {args.input}...")
    try:
        pcd = o3d.io.read_point_cloud(args.input)
    except Exception as e:
        print(f"Failed to read PLY file with Open3D: {e}")
        sys.exit(1)
        
    if not pcd.has_points():
        print("Error: Point cloud is empty.")
        sys.exit(1)
        
    points = np.asarray(pcd.points)
    colors = None
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        
    print(f"Converting to GLB...")
    write_glb(points, colors, output_path)

if __name__ == "__main__":
    main()
