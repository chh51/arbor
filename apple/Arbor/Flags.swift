// @file Flags.swift
// Project: Arbor
//
// Copyright (C) 2026 Jean-Romain Roussel (r-lidar) <info @ r-lidar.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

import Foundation

/// One exterior ring. Coordinates are `(x, y)` in the cloud's CRS.
public struct Polygon: Sendable {
    public var exterior: [(x: Double, y: Double)]

    public init(exterior: [(x: Double, y: Double)]) {
        self.exterior = exterior
    }
}

/// Flag trees that should stay out of later stages.
///
/// Writes `UserData`. A tree stays ``ARBORTREE`` only when its points extend from
/// below `maxHeight` to above it (`hag` max greater than `maxHeight`, and `hag`
/// min less than `maxHeight`). Every other tree that is not ``ARBORLOW`` becomes
/// ``ARBORUNDERSTORY``. An earlier understory flag is cleared first. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/).
///
/// - Parameter cloud: A cloud with `treeID` and `hag`.
/// - Parameter maxHeight: Metres. Default 2.
/// - Returns: The same cloud.
///
/// ## See Also
/// ``qsf(_:minHeight:params:)``
/// ``flagBuffer(_:seeds:buffer:)``
public func flagSmallTrees(_ cloud: PointCloud, maxHeight: Double = 2) throws -> PointCloud {
    try cloud.validate()
    let treeID = try cloud.requireTreeID()
    let hag = try cloud.requireHag()

    struct Span {
        var minH: Double
        var maxH: Double
    }
    var spans: [Int32: Span] = [:]
    for index in 0..<cloud.count {
        let id = treeID[index]
        guard id > 0 else { continue }
        let height = hag[index]
        if var span = spans[id] {
            span.minH = min(span.minH, height)
            span.maxH = max(span.maxH, height)
            spans[id] = span
        } else {
            spans[id] = Span(minH: height, maxH: height)
        }
    }
    let kept = Set(spans.compactMap { id, span in
        (span.maxH > maxHeight && span.minH < maxHeight) ? id : nil
    })

    var userData = cloud.userData ?? Array(repeating: ARBORTREE, count: cloud.count)
    for index in userData.indices where userData[index] == ARBORUNDERSTORY {
        userData[index] = ARBORTREE
    }
    for index in 0..<cloud.count where !kept.contains(treeID[index]) && userData[index] != ARBORLOW {
        userData[index] = ARBORUNDERSTORY
    }
    cloud.userData = userData
    return cloud
}

/// Flag trees whose seed falls outside an inward buffer of the convex hull.
///
/// `buffer` is metres and must be negative. The default, `-5`, drops trees within
/// 5 m of the hull. When `seeds` is omitted, each tree's seed is the mean of its
/// lowest points (up to 100, and at least 10). A tree that is not ``ARBORLOW``
/// and whose seed is outside the buffered hull becomes ``ARBORBUFFER``. An earlier
/// buffer flag is cleared first. If the buffer erases the hull, or no seed remains
/// inside, the result is an empty cloud. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/).
///
/// - Parameter cloud: A cloud with `treeID`.
/// - Parameter seeds: Optional seed cloud with `treeID`.
/// - Parameter buffer: Negative distance, metres. Default `-5`.
/// - Returns: The same cloud, or an empty cloud when nothing is inside the buffer.
///
/// ## See Also
/// ``flagSmallTrees(_:maxHeight:)``
/// ``findSeeds(_:params:)``
public func flagBuffer(_ cloud: PointCloud, seeds: PointCloud? = nil, buffer: Double = -5) throws -> PointCloud {
    if buffer > 0 {
        throw ArborError.invalidArgument("'buffer' must be negative.")
    }
    let hull = try convexHull(of: cloud)
    guard let region = insetConvex(hull, distance: -buffer) else {
        return cloud.emptied()
    }
    return try applyBuffer(cloud, seeds: seeds, regions: [region])
}

/// Flag trees whose seed falls outside `polygons`.
///
/// Same `UserData` rules as ``flagBuffer(_:seeds:buffer:)``. A seed is kept when
/// it lies in any polygon. The rings are used as given. They are not buffered again.
///
/// - Parameter cloud: A cloud with `treeID`.
/// - Parameter seeds: Optional seed cloud with `treeID`.
/// - Parameter polygons: Exterior rings. A point inside any ring is kept.
/// - Returns: The same cloud, or an empty cloud when no seed is inside.
public func flagBuffer(_ cloud: PointCloud, seeds: PointCloud? = nil, polygons: [Polygon]) throws -> PointCloud {
    let regions = polygons.map { polygon in
        polygon.exterior.map { ($0.x, $0.y) }
    }
    return try applyBuffer(cloud, seeds: seeds, regions: regions)
}

private func applyBuffer(_ cloud: PointCloud, seeds: PointCloud?, regions: [[(Double, Double)]]) throws -> PointCloud {
    try cloud.validate()
    let treeID = try cloud.requireTreeID()
    if let seeds {
        try seeds.validate()
        guard seeds.treeID != nil else { throw ArborError.missingAttribute("treeID") }
    }

    let roots = try rootsByTree(cloud: cloud, seeds: seeds)
    if roots.isEmpty {
        return cloud.emptied()
    }
    let inside = Set(roots.compactMap { id, root in
        regions.contains { ringContains($0, x: root.x, y: root.y) } ? id : nil
    })
    if inside.isEmpty {
        return cloud.emptied()
    }

    var userData = cloud.userData ?? Array(repeating: ARBORTREE, count: cloud.count)
    for index in userData.indices where userData[index] == ARBORBUFFER {
        userData[index] = ARBORTREE
    }
    for index in 0..<cloud.count where !inside.contains(treeID[index]) && userData[index] != ARBORLOW {
        userData[index] = ARBORBUFFER
    }
    cloud.userData = userData
    return cloud
}

private struct Root {
    var x: Double
    var y: Double
    var z: Double
}

private func rootsByTree(cloud: PointCloud, seeds: PointCloud?) throws -> [Int32: Root] {
    if let seeds {
        guard let ids = seeds.treeID else { throw ArborError.missingAttribute("treeID") }
        var groups: [Int32: (sx: Double, sy: Double, sz: Double, n: Int)] = [:]
        for index in 0..<seeds.count where ids[index] > 0 {
            let id = ids[index]
            var group = groups[id] ?? (0, 0, 0, 0)
            group.sx += seeds.x[index]
            group.sy += seeds.y[index]
            group.sz += seeds.z[index]
            group.n += 1
            groups[id] = group
        }
        return groups.compactMapValues { group in
            guard group.n > 0 else { return nil }
            let root = Root(x: group.sx / Double(group.n), y: group.sy / Double(group.n), z: group.sz / Double(group.n))
            return (root.x.isFinite && root.y.isFinite) ? root : nil
        }
    }

    guard let ids = cloud.treeID else { throw ArborError.missingAttribute("treeID") }
    var groups: [Int32: [(Double, Double, Double)]] = [:]
    for index in 0..<cloud.count where ids[index] > 0 {
        groups[ids[index], default: []].append((cloud.x[index], cloud.y[index], cloud.z[index]))
    }
    var roots: [Int32: Root] = [:]
    for (id, points) in groups {
        let ordered = points.sorted { $0.2 < $1.2 }
        let take = min(100, ordered.count)
        guard take >= 10 else { continue }
        let sample = ordered.prefix(take)
        let n = Double(sample.count)
        let root = Root(
            x: sample.reduce(0) { $0 + $1.0 } / n,
            y: sample.reduce(0) { $0 + $1.1 } / n,
            z: sample.reduce(0) { $0 + $1.2 } / n
        )
        if root.x.isFinite && root.y.isFinite {
            roots[id] = root
        }
    }
    return roots
}

private func convexHull(of cloud: PointCloud) throws -> [(Double, Double)] {
    try cloud.validate()
    var points = [(Double, Double)]()
    points.reserveCapacity(cloud.count)
    for index in 0..<cloud.count where cloud.x[index].isFinite && cloud.y[index].isFinite {
        points.append((cloud.x[index], cloud.y[index]))
    }
    points.sort { lhs, rhs in
        lhs.0 == rhs.0 ? lhs.1 < rhs.1 : lhs.0 < rhs.0
    }
    var unique: [(Double, Double)] = []
    for point in points where unique.last?.0 != point.0 || unique.last?.1 != point.1 {
        unique.append(point)
    }
    guard unique.count >= 3 else { return unique }

    func cross(_ o: (Double, Double), _ a: (Double, Double), _ b: (Double, Double)) -> Double {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    }
    var lower: [(Double, Double)] = []
    for point in unique {
        while lower.count >= 2 && cross(lower[lower.count - 2], lower[lower.count - 1], point) <= 0 {
            lower.removeLast()
        }
        lower.append(point)
    }
    var upper: [(Double, Double)] = []
    for point in unique.reversed() {
        while upper.count >= 2 && cross(upper[upper.count - 2], upper[upper.count - 1], point) <= 0 {
            upper.removeLast()
        }
        upper.append(point)
    }
    lower.removeLast()
    upper.removeLast()
    return lower + upper
}

private func insetConvex(_ hull: [(Double, Double)], distance: Double) -> [(Double, Double)]? {
    if distance < 0 { return nil }
    if distance == 0 { return hull.count >= 3 ? hull : nil }
    var polygon = hull
    if polygon.count < 3 { return nil }
    if signedArea(polygon) < 0 {
        polygon.reverse()
    }
    let count = polygon.count
    var shifted: [(SIMD2<Double>, SIMD2<Double>)] = []
    shifted.reserveCapacity(count)
    for index in 0..<count {
        let a = SIMD2(polygon[index].0, polygon[index].1)
        let b = SIMD2(polygon[(index + 1) % count].0, polygon[(index + 1) % count].1)
        let edge = b - a
        let length = (edge.x * edge.x + edge.y * edge.y).squareRoot()
        if length == 0 { return nil }
        let inward = SIMD2(-edge.y / length, edge.x / length) * distance
        shifted.append((a + inward, b + inward))
    }
    var inset: [(Double, Double)] = []
    inset.reserveCapacity(count)
    for index in 0..<count {
        let current = shifted[index]
        let next = shifted[(index + 1) % count]
        guard let point = lineIntersection(current.0, current.1, next.0, next.1) else { return nil }
        inset.append((point.x, point.y))
    }
    if signedArea(inset) <= 1e-8 { return nil }
    for vertex in inset {
        for edge in 0..<count {
            let gap = distanceToSegment(vertex, polygon[edge], polygon[(edge + 1) % count])
            if gap + 1e-6 < distance { return nil }
        }
    }
    return inset
}

private func signedArea(_ ring: [(Double, Double)]) -> Double {
    var sum = 0.0
    for index in ring.indices {
        let next = ring[(index + 1) % ring.count]
        sum += ring[index].0 * next.1 - next.0 * ring[index].1
    }
    return sum / 2
}

private func lineIntersection(_ a: SIMD2<Double>, _ b: SIMD2<Double>, _ c: SIMD2<Double>, _ d: SIMD2<Double>) -> SIMD2<Double>? {
    let r = b - a
    let s = d - c
    let denom = r.x * s.y - r.y * s.x
    if abs(denom) < 1e-12 { return nil }
    let t = ((c.x - a.x) * s.y - (c.y - a.y) * s.x) / denom
    return a + r * t
}

private func distanceToSegment(_ point: (Double, Double), _ a: (Double, Double), _ b: (Double, Double)) -> Double {
    let abx = b.0 - a.0
    let aby = b.1 - a.1
    let length2 = abx * abx + aby * aby
    if length2 == 0 {
        let dx = point.0 - a.0
        let dy = point.1 - a.1
        return (dx * dx + dy * dy).squareRoot()
    }
    var t = ((point.0 - a.0) * abx + (point.1 - a.1) * aby) / length2
    t = min(1, max(0, t))
    let dx = point.0 - (a.0 + t * abx)
    let dy = point.1 - (a.1 + t * aby)
    return (dx * dx + dy * dy).squareRoot()
}

private func ringContains(_ ring: [(Double, Double)], x: Double, y: Double) -> Bool {
    guard ring.count >= 3 else { return false }
    var inside = false
    var previous = ring.count - 1
    for index in ring.indices {
        let yi = ring[index].1
        let yj = ring[previous].1
        let xi = ring[index].0
        let xj = ring[previous].0
        if (yi > y) != (yj > y) {
            let cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < cross {
                inside.toggle()
            }
        }
        previous = index
    }
    return inside
}
