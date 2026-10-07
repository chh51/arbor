// @file Oversegmentation.swift
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

/// Merge tree ids that instance segmentation split apart.
///
/// ``segmentInstance(_:seeds:params:)`` already runs this when
/// `params.instance.oversegmentationSolverEnabled` is true. Calling it again
/// remaps `treeID` where several ids share one stem. Detection walks height
/// slices, fits a circle to each cluster, and collapses id chains that the
/// circles support.
///
/// - Parameter cloud: A cloud that already has `treeID` and `hag`.
/// - Returns: The same cloud, with merged ids written in place.
///
/// ## See Also
/// ``segmentInstance(_:seeds:params:)``
public func resolveOversegmentation(_ cloud: PointCloud) throws -> PointCloud {
    try cloud.validate()
    return try Engine.resolveOversegmentation(cloud)
}
