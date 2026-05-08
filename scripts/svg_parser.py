#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np

try:
    from svgpathtools import svg2paths, Line, QuadraticBezier, CubicBezier, Arc
except ImportError:
    print("[svg_parser] svgpathtools가 설치되지 않았습니다.")
    print("  pip install svgpathtools")
    sys.exit(1)


def parse_svg(svg_file, scale_m_per_px, samples_per_segment=15):
    """
    SVG 파일의 <path> 요소를 파싱하여 드론 로컬 좌표계(미터)의 waypoint 리스트를 반환.

    좌표 변환 규칙:
      - SVG (0,0) = 좌측 상단, y 증가 = 아래
      - 드론 로컬 프레임: 원점 = 이륙 지점, x = East, y = North (y 증가 = 앞)
      - 변환: 경로 중심을 (0,0)으로 이동 → 스케일 적용 → y축 반전

    Args:
        svg_file        : SVG 파일 경로
        scale_m_per_px  : 1 픽셀이 몇 미터인지 (예: 0.1 → 100px = 10m)
        samples_per_segment: 베지에/호 세그먼트당 샘플 점 수

    Returns:
        List of (x, y) tuples in meters (중심 기준, y축 반전 적용)
    """
    paths, _ = svg2paths(svg_file)

    raw_points = []
    for path in paths:
        for segment in path:
            pts = _sample_segment(segment, samples_per_segment)
            raw_points.extend(pts)

    if not raw_points:
        return []

    arr = np.array(raw_points, dtype=float)  # (N, 2): [px_x, px_y]

    # 중심을 원점으로 이동
    cx = (arr[:, 0].max() + arr[:, 0].min()) / 2.0
    cy = (arr[:, 1].max() + arr[:, 1].min()) / 2.0
    arr[:, 0] -= cx
    arr[:, 1] -= cy

    # 픽셀 → 미터 스케일
    arr *= scale_m_per_px

    # SVG y축(아래가 +) → 드론 y축(앞이 +) 반전
    arr[:, 1] *= -1

    return [(float(p[0]), float(p[1])) for p in arr]


def _sample_segment(segment, n):
    """세그먼트를 n+1개의 점으로 균등 샘플링."""
    pts = []
    for i in range(n + 1):
        t = i / float(n)
        pt = segment.point(t)
        pts.append((pt.real, pt.imag))
    return pts


# --- 독립 실행 테스트 ---
if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python svg_parser.py <svg_file> <scale_m_per_px>")
        sys.exit(1)

    svg_path = sys.argv[1]
    scale = float(sys.argv[2])

    waypoints = parse_svg(svg_path, scale)
    print("파싱 완료: %d개 waypoint" % len(waypoints))
    if waypoints:
        xs = [p[0] for p in waypoints]
        ys = [p[1] for p in waypoints]
        print("  X 범위: [%.2f, %.2f] m" % (min(xs), max(xs)))
        print("  Y 범위: [%.2f, %.2f] m" % (min(ys), max(ys)))
        print("  첫 5개:", waypoints[:5])
