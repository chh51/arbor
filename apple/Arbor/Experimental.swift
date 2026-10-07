// @file Experimental.swift
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

/// Which tree ``extractTreeContext(_:tree:excludeTree:)`` gathers neighbours for.
public enum TreeReference {
    /// A `treeID` stored on the cloud.
    case id(Int)
    /// A cloud of that one tree. Its first `treeID` is used.
    case cloud(PointCloud)
}

/// Points of the trees that surround one tree.
///
/// Experimental. The R package warns that this may be removed. The cloud must
/// have `treeID` and must not be empty.
///
/// - Parameter tree: A tree id, or a cloud of that tree.
/// - Parameter excludeTree: Drop the target tree from the result. Default false.
/// - Returns: A new cloud of the neighbouring points.
public func extractTreeContext(_ cloud: PointCloud, tree: TreeReference, excludeTree: Bool = false) throws -> PointCloud {
    try cloud.validate()
    guard cloud.count > 0 else {
        throw ArborError.invalidArgument("'las' is empty")
    }
    guard cloud.treeID != nil else {
        throw ArborError.missingAttribute("treeID")
    }
    let id: Int
    switch tree {
    case .id(let treeID):
        id = treeID
    case .cloud(let treeCloud):
        guard let first = treeCloud.treeID?.first else {
            throw ArborError.missingAttribute("treeID")
        }
        id = Int(first)
    }
    return try Engine.extractTreeContext(cloud, treeID: id, excludeTree: excludeTree)
}

/// Label wood and foliage on one tree from its QSM.
///
/// Experimental. A point is wood when its distance to a cylinder is less than
/// `radius * 1.3 + 0.02`. `foliage` is then `0` for wood and `1` for foliage.
///
/// - Parameter cloud: Points of one tree.
/// - Parameter forest: The models for that cloud. A single ``QSM`` can be wrapped in a ``QSF``.
/// - Returns: The same cloud, with `foliage` written in place.
public func qsfSegmentSemantic(_ cloud: PointCloud, forest: QSF) throws -> PointCloud {
    try cloud.validate()
    let distances = try Engine.qsmDistances(forest, cloud)
    guard distances.count == cloud.count else {
        throw ArborError.invalidArgument("distance table length does not match the point cloud")
    }
    var foliage = [Int32](repeating: 1, count: cloud.count)
    for index in 0..<cloud.count {
        let sample = distances[index]
        if sample.distance < sample.radius * 1.3 + 0.02 {
            foliage[index] = 0
        }
    }
    cloud.foliage = foliage
    return cloud
}
