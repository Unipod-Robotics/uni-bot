#!/usr/bin/env python3
"""
Generate ArUco Marker Images

Creates printable ArUco marker PNG images for the DICT_4X4_50 dictionary.
These can be printed for real robot testing or used as textures in Gazebo.

Usage:
    python3 generate_markers.py [--size SIZE_MM] [--count COUNT]
"""

import argparse
import os
import cv2
import numpy as np


def generate_markers(output_dir: str, size_pixels: int = 200, count: int = 5):
    """Generate ArUco marker images."""
    
    # Use DICT_4X4_50 dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    os.makedirs(output_dir, exist_ok=True)
    
    for marker_id in range(count):
        # Generate marker image - handle different OpenCV versions
        try:
            # OpenCV 4.7+
            marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_pixels)
        except AttributeError:
            # Older OpenCV - use drawMarker
            marker_image = np.zeros((size_pixels, size_pixels), dtype=np.uint8)
            marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, size_pixels, marker_image, 1)
        
        # Add white border (important for detection)
        border_size = size_pixels // 4
        bordered = cv2.copyMakeBorder(
            marker_image,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT,
            value=255
        )
        
        # Save as PNG
        filename = os.path.join(output_dir, f'marker_{marker_id}.png')
        cv2.imwrite(filename, bordered)
        print(f'Generated: {filename}')
    
    print(f'\nGenerated {count} markers in {output_dir}')
    print(f'Marker size: {size_pixels}px (print at desired physical size)')


def main():
    parser = argparse.ArgumentParser(description='Generate ArUco marker images')
    parser.add_argument('--size', type=int, default=200,
                       help='Marker size in pixels (default: 200)')
    parser.add_argument('--count', type=int, default=5,
                       help='Number of markers to generate (default: 5)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output directory (default: ./markers)')
    
    args = parser.parse_args()
    
    # Default output directory
    if args.output is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, '..', 'markers')
    else:
        output_dir = args.output
    
    generate_markers(output_dir, args.size, args.count)


if __name__ == '__main__':
    main()
