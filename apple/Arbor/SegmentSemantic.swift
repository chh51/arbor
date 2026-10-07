// @file SegmentSemantic.swift
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

/// Wood-foliage semantic segmentation.
///
/// Builds a k-nearest-neighbour network and finds the least-cost path from each
/// point to the ground. A point is wood when it lies near that skeleton or its
/// wood likelihood is high enough. Run ``woodLikelihood(_:params:)`` first. The
/// cloud needs `hag`. Small wood clusters are removed. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/) for more details.
///
/// - Parameter cloud: The cloud to classify.
/// - Parameter params: See ``arborParametersDefault``.
/// - Returns: The same cloud. `foliage` is `0` for wood and `1` or `2` for foliage.
///   `UserData` is `ARBORLOW` below the cut and `ARBORTREE` otherwise.
///
/// ## See Also
/// ``woodLikelihood(_:params:)``
/// ``segmentGround(_:params:)``
public func segmentSemantic(_ cloud: PointCloud, params: ArborParameters = arborParametersDefault) throws -> PointCloud {
    try cloud.validate()
    return try Engine.segmentSemantic(cloud, params)
}
