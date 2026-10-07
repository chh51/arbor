// @file Seeds.swift
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

/// Find seeds for ``segmentInstance(_:seeds:params:)``.
///
/// Each seed carries a reference `treeID`. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/) for more details.
///
/// - Parameter cloud: The cloud to search.
/// - Parameter params: See ``arborParametersDefault``.
/// - Returns: A new cloud of seed points. The input cloud is left unchanged.
public func findSeeds(_ cloud: PointCloud, params: ArborParameters = arborParametersDefault) throws -> PointCloud {
    try cloud.validate()
    let seeds = try Engine.findSeeds(cloud, params)
    if seeds.crs.isEmpty {
        seeds.crs = cloud.crs
    }
    return seeds
}
