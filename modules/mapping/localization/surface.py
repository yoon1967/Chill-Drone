"""Observed voxel centroids -> normal-aware graph -> bounded ball-pivoting surface."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def aggregate_voxels(xyz, rgb, cell_size):
    """Average only observed samples in each fixed world-aligned cell; never fill empty cells."""
    xyz, rgb = np.asarray(xyz, dtype=float), np.asarray(rgb, dtype=float)
    if not np.isfinite(cell_size) or cell_size <= 0 or xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("Invalid voxel size/points")
    if len(xyz) != len(rgb) or not np.isfinite(xyz).all() or not np.isfinite(rgb).all():
        raise ValueError("Invalid point colors/coordinates")
    cells, inverse, counts = np.unique(np.floor(xyz / cell_size).astype(np.int64), axis=0,
                                       return_inverse=True, return_counts=True)
    positions, colors = np.zeros((len(cells), 3)), np.zeros((len(cells), 3))
    np.add.at(positions, inverse, xyz)
    np.add.at(colors, inverse, rgb)
    return positions / counts[:, None], colors / counts[:, None], cells, counts


def build_surface(scene, voxel_size=None, max_error=1.5, min_track=3):
    started = time.perf_counter()
    raw = np.asarray(scene["points"], dtype=float)
    if raw.ndim != 2 or raw.shape[1] != 9 or len(raw) < 10 or not np.isfinite(raw).all():
        raise ValueError("At least ten finite SfM point rows are required")
    selected = raw[(raw[:, 7] <= max_error) & (raw[:, 8] >= min_track)]
    if len(selected) < 10:
        raise ValueError("Insufficient supported points after reprojection/track filtering")
    diagonal = float(np.linalg.norm(np.ptp(selected[:, 1:4], axis=0)))
    cell = voxel_size if voxel_size is not None else (0.03 if scene["scale"]["unit"] == "m" else diagonal / 200)
    points, colors, cells, counts = aggregate_voxels(selected[:, 1:4], selected[:, 4:7], cell)
    if len(points) < 10:
        raise ValueError("Voxel size leaves fewer than ten surface nodes")
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors / 255)
    spacing = float(np.median(cKDTree(points).query(points, k=2)[0][:, 1]))
    base = max(cell, spacing)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=base * 5, max_nn=30))
    normals = np.asarray(cloud.normals)
    camera_positions = np.asarray([c["position"] for c in scene["cameras"]])
    if len(camera_positions):
        closest = camera_positions[cKDTree(camera_positions).query(points)[1]]
        normals[np.einsum("ij,ij->i", normals, closest - points) < 0] *= -1
    cloud.normals = o3d.utility.Vector3dVector(normals)
    # A local radius and normal agreement prevent a global triangulation across unknown gaps.
    max_edge = base * 4
    neighbors = cKDTree(points).query_pairs(max_edge, output_type="ndarray")
    graph = []
    degree = np.zeros(len(points), dtype=int)
    if len(neighbors):
        order = np.argsort(np.linalg.norm(points[neighbors[:, 0]] - points[neighbors[:, 1]], axis=1))
        for a, b in neighbors[order]:
            if degree[a] >= 8 or degree[b] >= 8 or np.dot(normals[a], normals[b]) < np.cos(np.radians(50)):
                continue
            graph.append([int(a), int(b)])
            degree[a] += 1
            degree[b] += 1
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud, o3d.utility.DoubleVector([base * 1.5, base * 2]))
    triangles = np.asarray(mesh.triangles).copy()
    if len(triangles):
        vertices = points[triangles]
        edge_lengths = np.stack([np.linalg.norm(vertices[:, i] - vertices[:, (i + 1) % 3], axis=1) for i in range(3)])
        areas = np.linalg.norm(np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0]), axis=1) / 2
        agreement = np.stack([np.einsum("ij,ij->i", normals[triangles[:, i]], normals[triangles[:, (i + 1) % 3]]) for i in range(3)])
        triangles = triangles[(edge_lengths.max(axis=0) <= max_edge) & (areas > cell ** 2 * 0.001) &
                              (agreement.min(axis=0) >= np.cos(np.radians(55)))]
    result = {"schema_version": "observed-surface/1.0", "method": "voxel_centroid+normal_graph+open3d_ball_pivoting",
              "scene_binding_sha256": hashlib.sha256(json.dumps({key: scene[key] for key in
                  ("points", "cameras", "scale")}, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
              "unit": scene["scale"]["unit"], "voxel_size": cell, "max_edge_length": max_edge,
              "positions": points.tolist(), "colors": np.rint(colors).astype(int).tolist(),
              "normals": normals.tolist(), "triangles": triangles.tolist(), "edges": graph,
              "cells": cells.tolist(), "sample_counts": counts.tolist(),
              "statistics": {"input_points": len(raw), "supported_points": len(selected), "nodes": len(points),
                             "triangles": len(triangles), "edges": len(graph),
                             "processing_seconds": time.perf_counter() - started},
              "occupancy": "observed_surface_only; all unobserved cells are unknown, not free",
              "texture": "vertex_colors_from_observed_points; no photo planes or photo textures",
              "navigation": "visual_inspection_only; not collision-certified or flight-safe"}
    validate_surface(result)
    return result


def validate_surface(surface):
    points = np.asarray(surface["positions"])
    triangles = np.asarray(surface["triangles"], dtype=int).reshape(-1, 3)
    edges = np.asarray(surface["edges"], dtype=int).reshape(-1, 2)
    if len(points) < 1 or not np.isfinite(points).all():
        raise ValueError("Invalid surface positions")
    for indices in (triangles, edges):
        if len(indices) and (indices.min() < 0 or indices.max() >= len(points)):
            raise ValueError("Surface index out of range")
    if np.asarray(surface["colors"]).shape != points.shape or np.asarray(surface["normals"]).shape != points.shape:
        raise ValueError("Surface color/normal count differs")
    if len(triangles):
        vertices = points[triangles]
        lengths = [np.linalg.norm(vertices[:, i] - vertices[:, (i + 1) % 3], axis=1) for i in range(3)]
        areas = np.linalg.norm(np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0]), axis=1)
        if np.max(lengths) > surface["max_edge_length"] * (1 + 1e-8) or np.any(areas <= 0):
            raise ValueError("Invalid or excessively long triangle")
    if len(edges) and np.max(np.linalg.norm(points[edges[:, 0]] - points[edges[:, 1]], axis=1)) > surface["max_edge_length"] * (1 + 1e-8):
        raise ValueError("Graph bridges beyond the allowed local radius")
    if len(surface["sample_counts"]) != len(points) or sum(surface["sample_counts"]) != surface["statistics"]["supported_points"]:
        raise ValueError("Voxel support counts differ")
    return {"nodes": len(points), "triangles": len(triangles), "edges": len(edges)}


def write_mesh_ply(path, surface):
    """Portable triangle mesh for Blender/MeshLab; no photos or invented textures."""
    validate_surface(surface)
    with Path(path).open("x", encoding="ascii", newline="\n") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(surface['positions'])}\n")
        stream.write("property double x\nproperty double y\nproperty double z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write(f"element face {len(surface['triangles'])}\nproperty list uchar int vertex_indices\nend_header\n")
        for xyz, rgb in zip(surface["positions"], surface["colors"]):
            stream.write(" ".join(map(str, [*xyz, *rgb])) + "\n")
        for face in surface["triangles"]:
            stream.write("3 " + " ".join(map(str, face)) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--voxel-size", type=float)
    parser.add_argument("--mesh-output", type=Path, help="Optional triangle PLY for Blender/MeshLab")
    args = parser.parse_args()
    if args.output.exists() or (args.mesh_output and args.mesh_output.exists()):
        parser.error("Select a new surface file; existing artifacts are preserved")
    surface = build_surface(json.loads(args.scene.read_text(encoding="utf-8")), args.voxel_size)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(surface, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    if args.mesh_output:
        args.mesh_output.parent.mkdir(parents=True, exist_ok=True)
        write_mesh_ply(args.mesh_output, surface)
    print(json.dumps(surface["statistics"]))
