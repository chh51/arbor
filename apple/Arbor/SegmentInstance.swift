// @file SegmentInstance.swift
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

/// Individual-tree instance segmentation.
///
/// Builds a point network and assigns each point the `treeID` of the seed reached
/// by the least-cost path. The cloud must already have `foliage` from
/// ``segmentSemantic(_:params:)``. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/) for more details.
///
/// - Parameter cloud: The cloud to segment.
/// - Parameter seeds: Seed points with `treeID`, usually from ``findSeeds(_:params:)``.
/// - Parameter params: See ``arborParametersDefault``.
/// - Returns: The same cloud, with `treeID` written on every point.
///
/// ## See Also
/// ``findSeeds(_:params:)``
/// ``segmentSemantic(_:params:)``
public func segmentInstance(_ cloud: PointCloud, seeds: PointCloud, params: ArborParameters = arborParametersDefault) throws -> PointCloud {
    try cloud.validate()
    try seeds.validate()
    return try Engine.segmentInstance(cloud, seeds, params)
}
