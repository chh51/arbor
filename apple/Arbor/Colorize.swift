// @file Colorize.swift
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

/// Assign one RGB colour per tree.
///
/// Foliage can be drawn darker than the wood of the same tree so semantic and
/// instance segmentation show in one colouring. Plot helpers stay in the R package.
///
/// - Parameter cloud: A cloud with `treeID`.
/// - Parameter darkenFoliage: Darken foliage relative to its tree. Default true.
/// - Returns: The same cloud, with `R`, `G`, and `B` written in place.
public func colorizeTrees(_ cloud: PointCloud, darkenFoliage: Bool = true) throws -> PointCloud {
    try cloud.validate()
    return try Engine.colorize(cloud, darkenFoliage: darkenFoliage)
}
